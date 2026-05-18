#!/usr/bin/env python3
"""
mission_planner.py - Nodo 5 del proyecto UAV Control - Challenge.

GUI PyQt5 que construye y ejecuta misiones. Incluye failsafe de batería:
cuando la batería cae por debajo del umbral (BATTERY_THRESHOLD = 30%) y el
dron está volando, envía un aterrizaje urgente por /tello_cmd_priority y
aborta la misión en curso.

Tópicos:
    Publica:
        /tello_cmd          comandos de la misión
        /tello_cmd_priority aterrizaje urgente por batería crítica
    Suscribe:
        /battery_status     indicador de batería + validación de despegue
        /drone_status       confirmación de takeoff/land
"""

import json
import sys
import threading
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32

from PyQt5.QtCore import Qt, QObject, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QListWidget, QListWidgetItem, QDialog,
    QComboBox, QSpinBox, QDoubleSpinBox, QDialogButtonBox, QTextEdit,
    QFrame, QAbstractItemView, QMessageBox, QFormLayout, QGroupBox
)
from PyQt5.QtGui import QFont


# ====================================================================
# CONSTANTES
# ====================================================================
BATTERY_THRESHOLD = 64          # Umbral de batería para aterrizaje de emergencia
STATUS_TIMEOUT = 10.0

# --- Comandos de un solo parámetro numérico ---
SINGLE_PARAM_COMMANDS = {
    'forward':    {'label': 'Adelante',        'unit': 'cm', 'min': 20, 'max': 500, 'default': 50},
    'back':       {'label': 'Atrás',           'unit': 'cm', 'min': 20, 'max': 500, 'default': 50},
    'left':       {'label': 'Izquierda',       'unit': 'cm', 'min': 20, 'max': 500, 'default': 50},
    'right':      {'label': 'Derecha',         'unit': 'cm', 'min': 20, 'max': 500, 'default': 50},
    'up':         {'label': 'Subir',           'unit': 'cm', 'min': 20, 'max': 500, 'default': 30},
    'down':       {'label': 'Bajar',           'unit': 'cm', 'min': 20, 'max': 500, 'default': 30},
    'rotate_cw':  {'label': 'Rotar Horario',   'unit': '°',  'min': 1,  'max': 360, 'default': 90},
    'rotate_ccw': {'label': 'Rotar Antihor.',  'unit': '°',  'min': 1,  'max': 360, 'default': 90},
}

# --- Comandos multi-parámetro ---
MULTI_PARAM_COMMANDS = {
    'go': {
        'label': 'Ir a (go x y z speed)',
        'params': [
            {'name': 'x',     'unit': 'cm',   'min': -500, 'max': 500, 'default': 0},
            {'name': 'y',     'unit': 'cm',   'min': -500, 'max': 500, 'default': 0},
            {'name': 'z',     'unit': 'cm',   'min': -500, 'max': 500, 'default': 50},
            {'name': 'speed', 'unit': 'cm/s', 'min': 10,   'max': 100, 'default': 50},
        ],
    },
    'curve': {
        'label': 'Curva (curve x1 y1 z1 x2 y2 z2 speed)',
        'params': [
            {'name': 'x1',    'unit': 'cm',   'min': -500, 'max': 500, 'default': 50},
            {'name': 'y1',    'unit': 'cm',   'min': -500, 'max': 500, 'default': 50},
            {'name': 'z1',    'unit': 'cm',   'min': -500, 'max': 500, 'default': 0},
            {'name': 'x2',    'unit': 'cm',   'min': -500, 'max': 500, 'default': 100},
            {'name': 'y2',    'unit': 'cm',   'min': -500, 'max': 500, 'default': 0},
            {'name': 'z2',    'unit': 'cm',   'min': -500, 'max': 500, 'default': 0},
            {'name': 'speed', 'unit': 'cm/s', 'min': 10,   'max': 60,  'default': 30},
        ],
    },
}

# --- Comando con parámetro de selección (combo) ---
CHOICE_COMMANDS = {
    'flip': {
        'label': 'Flip',
        'choices': [
            {'value': 'f', 'label': 'Adelante (f)'},
            {'value': 'b', 'label': 'Atrás (b)'},
            {'value': 'l', 'label': 'Izquierda (l)'},
            {'value': 'r', 'label': 'Derecha (r)'},
        ],
        'default': 'f',
    },
}

# --- Comandos sin parámetros (acción instantánea) ---
NO_PARAM_COMMANDS = {
    'stop':      {'label': 'Hover (stop)'},
    'emergency': {'label': 'Emergencia (parar motores)'},
}

# Catálogo unificado para lookup rápido
ALL_COMMANDS = {}
for cmd, info in SINGLE_PARAM_COMMANDS.items():
    ALL_COMMANDS[cmd] = {'type': 'single', **info}
for cmd, info in MULTI_PARAM_COMMANDS.items():
    ALL_COMMANDS[cmd] = {'type': 'multi', **info}
for cmd, info in CHOICE_COMMANDS.items():
    ALL_COMMANDS[cmd] = {'type': 'choice', **info}
for cmd, info in NO_PARAM_COMMANDS.items():
    ALL_COMMANDS[cmd] = {'type': 'none', **info}

TAKEOFF_STEP = {'cmd': 'takeoff', 'value': None, 'fixed': True, 'label': 'Despegue'}
LAND_STEP    = {'cmd': 'land',    'value': None, 'fixed': True, 'label': 'Aterrizaje'}


# ====================================================================
# NODO ROS 2
# ====================================================================
class MissionPlannerNode(Node, QObject):
    log_signal      = pyqtSignal(str, str)
    status_signal   = pyqtSignal(str)
    battery_signal  = pyqtSignal(int)
    finished_signal = pyqtSignal(str)

    def __init__(self):
        QObject.__init__(self)
        Node.__init__(self, 'mission_planner')

        self.cmd_pub      = self.create_publisher(String, '/tello_cmd',          10)
        self.priority_pub = self.create_publisher(String, '/tello_cmd_priority', 10)

        self.battery_level = None
        self.drone_status = 'DISCONNECTED'
        self.land_already_sent = False
        self.battery_threshold = BATTERY_THRESHOLD

        self.create_subscription(Int32,  '/battery_status', self.battery_cb,      10)
        self.create_subscription(String, '/drone_status',   self.drone_status_cb, 10)

        self.mission_steps = []
        self.step_wait_s = 4.0
        self.current_step = 0
        self.mission_running = False
        self.mission_paused = False
        self.step_command_sent = False
        self.step_start_time = None

        self.create_timer(0.1, self.tick)   # 10 Hz

    # =================================================================
    # CALLBACKS
    # =================================================================
    def battery_cb(self, msg: Int32):
        self.battery_level = msg.data
        self.battery_signal.emit(msg.data)
        self._check_battery_failsafe()

    def drone_status_cb(self, msg: String):
        previous = self.drone_status
        if msg.data != self.drone_status:
            self.status_signal.emit(msg.data)
        self.drone_status = msg.data
        if previous == 'FLYING' and self.drone_status == 'LANDED':
            self.land_already_sent = False

    def _check_battery_failsafe(self):
        if (self.drone_status == 'FLYING'
                and self.battery_level <= self.battery_threshold
                and not self.land_already_sent):
            cmd = String()
            cmd.data = json.dumps({'cmd': 'land'})
            self.priority_pub.publish(cmd)
            self.land_already_sent = True

            if self.mission_running:
                self.mission_running = False
                self.mission_paused = False
                self.log_signal.emit('ERROR',
                    f'ABORTO: batería crítica ({self.battery_level}%). '
                    f'Aterrizaje de emergencia enviado.')
                self.finished_signal.emit('aborted')
            else:
                self.get_logger().warn(
                    f'Batería crítica ({self.battery_level}%). '
                    f'Aterrizaje de emergencia enviado.')

    # =================================================================
    # API DE LA GUI
    # =================================================================
    def start_mission(self, steps, wait_s):
        if self.battery_level is None:
            self.log_signal.emit('ERROR',
                'No hay lectura de batería todavía. Misión cancelada.')
            self.finished_signal.emit('aborted')
            return
        if self.battery_level <= self.battery_threshold:
            self.log_signal.emit('ERROR',
                f'Batería insuficiente ({self.battery_level}% <= {self.battery_threshold}%). '
                f'Misión cancelada.')
            self.finished_signal.emit('aborted')
            return

        self.mission_steps = list(steps)
        self.step_wait_s = float(wait_s)
        self.current_step = 0
        self.step_command_sent = False
        self.step_start_time = self.get_clock().now()
        self.mission_running = True
        self.mission_paused = False
        self.log_signal.emit('INFO',
            f'Misión iniciada con {len(self.mission_steps)} paso(s).')

    def pause_mission(self):
        if self.mission_running and not self.mission_paused:
            self.mission_paused = True
            self.log_signal.emit('INFO',
                'Misión pausada. El dron permanece en hover.')

    def resume_mission(self):
        if self.mission_running and self.mission_paused:
            self.mission_paused = False
            self.step_start_time = self.get_clock().now()
            self.log_signal.emit('INFO', 'Misión reanudada.')

    def stop_mission(self):
        if self.mission_running:
            self.mission_running = False
            cmd = String()
            cmd.data = json.dumps({'cmd': 'land'})
            self.priority_pub.publish(cmd)
            self.log_signal.emit('WARN',
                'STOP solicitado. Aterrizaje de emergencia enviado.')
            self.finished_signal.emit('aborted')

    # =================================================================
    # LÓGICA INTERNA
    # =================================================================
    def _send(self, cmd: str, value):
        payload = {'cmd': cmd}
        if value is not None:
            payload['value'] = value
        m = String()
        m.data = json.dumps(payload)
        self.cmd_pub.publish(m)
        self.log_signal.emit('CMD', f'>>> {payload}')

    def tick(self):
        if not self.mission_running or self.mission_paused:
            return

        if self.current_step >= len(self.mission_steps):
            self.log_signal.emit('OK', 'Misión completada con éxito ✔')
            self.mission_running = False
            self.finished_signal.emit('completed')
            return

        step = self.mission_steps[self.current_step]
        cmd = step['cmd']
        value = step.get('value')
        now = self.get_clock().now()

        if not self.step_command_sent:
            human = self._humanize(step)
            self.log_signal.emit('INFO',
                f'Ejecutando paso {self.current_step + 1}: {human}')
            self._send(cmd, value)
            self.step_command_sent = True
            self.step_start_time = now
            return

        elapsed = (now - self.step_start_time).nanoseconds / 1e9

        if cmd == 'takeoff':
            if self.drone_status == 'FLYING':
                self.log_signal.emit('OK',
                    f'Despegado correctamente (FLYING en {elapsed:.1f}s).')
                self._advance_step(now)
                return
            if elapsed > STATUS_TIMEOUT:
                self.log_signal.emit('ERROR',
                    f'Takeoff sin confirmación en {STATUS_TIMEOUT}s. Aborto.')
                self.mission_running = False
                self._send('land', None)
                self.finished_signal.emit('aborted')
                return

        elif cmd == 'land':
            if self.drone_status == 'LANDED':
                self.log_signal.emit('OK',
                    f'Aterrizado correctamente (LANDED en {elapsed:.1f}s).')
                self._advance_step(now)
                return
            if elapsed > STATUS_TIMEOUT:
                self.log_signal.emit('ERROR',
                    f'Land sin confirmación en {STATUS_TIMEOUT}s.')
                self.mission_running = False
                self.finished_signal.emit('aborted')
                return

        elif cmd == 'emergency':
            # Emergency es instantáneo, avanzar tras breve espera
            if elapsed >= 1.0:
                self.log_signal.emit('WARN', 'Emergency enviado.')
                self._advance_step(now)
                return

        elif cmd == 'stop':
            # Stop (hover) es instantáneo
            if elapsed >= 1.0:
                self.log_signal.emit('OK', 'Hover activado.')
                self._advance_step(now)
                return
        else:
            # Comandos de movimiento normales
            if self.drone_status == 'LANDED':
                self.log_signal.emit('ERROR',
                    'Aterrizaje inesperado durante el movimiento.')
                self.mission_running = False
                self.finished_signal.emit('aborted')
                return
            if elapsed >= self.step_wait_s:
                self.log_signal.emit('OK',
                    f'Paso {self.current_step + 1} completado.')
                self._advance_step(now)
                return

    def _advance_step(self, now):
        self.current_step += 1
        self.step_command_sent = False
        self.step_start_time = now

    def _humanize(self, step):
        cmd = step['cmd']
        value = step.get('value')

        if cmd == 'takeoff':
            return 'Despegando...'
        if cmd == 'land':
            return 'Aterrizando...'
        if cmd == 'emergency':
            return '⚠ EMERGENCIA — Parando motores'
        if cmd == 'stop':
            return 'Hover (detenerse en el aire)'

        info = ALL_COMMANDS.get(cmd)
        if not info:
            return f'{cmd} {value}'

        ctype = info['type']

        if ctype == 'single':
            return f'{info["label"]} {value} {info["unit"]}'

        if ctype == 'choice':
            choice_label = value
            for ch in info.get('choices', []):
                if ch['value'] == value:
                    choice_label = ch['label']
                    break
            return f'{info["label"]} → {choice_label}'

        if ctype == 'multi':
            # value es un dict con los nombres de los params
            parts = []
            for p in info['params']:
                v = value.get(p['name'], 0) if isinstance(value, dict) else 0
                parts.append(f'{p["name"]}={v}')
            return f'{info["label"]}  ({", ".join(parts)})'

        return f'{cmd} {value}'


# ====================================================================
# DIÁLOGO PARA AÑADIR UN PASO
# ====================================================================
class AddStepDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle('Añadir paso')
        self.setMinimumWidth(380)

        self._param_widgets = []   # lista de (nombre, widget)

        layout = QVBoxLayout(self)

        # --- Selector de comando ---
        layout.addWidget(QLabel('Tipo de movimiento:'))
        self.combo = QComboBox()

        # Grupo: Movimiento simple
        self.combo.addItem('── Movimiento simple ──', None)
        idx = self.combo.count() - 1
        self.combo.model().item(idx).setEnabled(False)
        for cmd, info in SINGLE_PARAM_COMMANDS.items():
            self.combo.addItem(f'  {info["label"]}', cmd)

        # Grupo: Rotación / Acción
        self.combo.addItem('── Acciones ──', None)
        idx = self.combo.count() - 1
        self.combo.model().item(idx).setEnabled(False)
        for cmd, info in CHOICE_COMMANDS.items():
            self.combo.addItem(f'  {info["label"]}', cmd)
        for cmd, info in NO_PARAM_COMMANDS.items():
            self.combo.addItem(f'  {info["label"]}', cmd)

        # Grupo: Avanzado (multi-parámetro)
        self.combo.addItem('── Avanzado ──', None)
        idx = self.combo.count() - 1
        self.combo.model().item(idx).setEnabled(False)
        for cmd, info in MULTI_PARAM_COMMANDS.items():
            self.combo.addItem(f'  {info["label"]}', cmd)

        self.combo.currentIndexChanged.connect(self._on_command_changed)
        layout.addWidget(self.combo)

        # --- Contenedor dinámico de parámetros ---
        self.params_group = QGroupBox('Parámetros')
        self.params_layout = QFormLayout()
        self.params_group.setLayout(self.params_layout)
        layout.addWidget(self.params_group)

        # --- Botones ---
        buttons = QDialogButtonBox(
            QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addWidget(buttons)

        # Seleccionar el primer comando real (index 1, después del header)
        self.combo.setCurrentIndex(1)

    # -----------------------------------------------------------------
    def _clear_params(self):
        while self.params_layout.count():
            item = self.params_layout.takeAt(0)
            w = item.widget()
            if w:
                w.deleteLater()
        self._param_widgets = []

    def _on_command_changed(self):
        cmd = self.combo.currentData()
        if cmd is None:
            return
        self._clear_params()

        info = ALL_COMMANDS.get(cmd)
        if not info:
            return

        ctype = info['type']

        if ctype == 'single':
            spin = QSpinBox()
            spin.setMinimum(info['min'])
            spin.setMaximum(info['max'])
            spin.setValue(info['default'])
            self.params_layout.addRow(f'{info["unit"]}:', spin)
            self._param_widgets = [('value', spin)]

        elif ctype == 'multi':
            for p in info['params']:
                spin = QSpinBox()
                spin.setMinimum(p['min'])
                spin.setMaximum(p['max'])
                spin.setValue(p['default'])
                label = f'{p["name"]} ({p["unit"]}):'
                self.params_layout.addRow(label, spin)
                self._param_widgets.append((p['name'], spin))

        elif ctype == 'choice':
            combo = QComboBox()
            for ch in info['choices']:
                combo.addItem(ch['label'], ch['value'])
            # Seleccionar default
            for i, ch in enumerate(info['choices']):
                if ch['value'] == info['default']:
                    combo.setCurrentIndex(i)
                    break
            self.params_layout.addRow('Dirección:', combo)
            self._param_widgets = [('choice', combo)]

        elif ctype == 'none':
            lbl = QLabel('(sin parámetros)')
            self.params_layout.addRow(lbl)
            self._param_widgets = []

    # -----------------------------------------------------------------
    def get_step(self):
        cmd = self.combo.currentData()
        info = ALL_COMMANDS.get(cmd)
        if not info:
            return None

        ctype = info['type']

        if ctype == 'single':
            _, spin = self._param_widgets[0]
            return {'cmd': cmd, 'value': spin.value(), 'fixed': False}

        elif ctype == 'multi':
            values = {}
            for name, spin in self._param_widgets:
                values[name] = spin.value()
            return {'cmd': cmd, 'value': values, 'fixed': False}

        elif ctype == 'choice':
            _, combo = self._param_widgets[0]
            return {'cmd': cmd, 'value': combo.currentData(), 'fixed': False}

        elif ctype == 'none':
            return {'cmd': cmd, 'value': None, 'fixed': False}

        return None


# ====================================================================
# WIDGET DE UN PASO
# ====================================================================
class StepWidget(QWidget):
    value_changed = pyqtSignal()
    delete_requested = pyqtSignal(object)

    def __init__(self, index: int, step: dict):
        super().__init__()
        self.step = step

        layout = QHBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4)

        # Número de paso
        self.num_label = QLabel(str(index))
        self.num_label.setFixedWidth(28)
        font = QFont()
        font.setBold(True)
        self.num_label.setFont(font)
        layout.addWidget(self.num_label)

        cmd = step['cmd']
        info = ALL_COMMANDS.get(cmd)

        # Nombre del comando
        if step.get('fixed'):
            label_text = step['label']
        elif info:
            label_text = info['label']
        else:
            label_text = cmd
        self.name_label = QLabel(label_text)
        self.name_label.setMinimumWidth(160)
        layout.addWidget(self.name_label)

        # Parámetros editables
        self._spins = []     # lista de (nombre, QSpinBox)
        self._combo = None   # QComboBox para choice

        if step.get('fixed'):
            layout.addWidget(QLabel('(fijo)'))

        elif info:
            ctype = info['type']

            if ctype == 'single':
                spin = QSpinBox()
                spin.setMinimum(info['min'])
                spin.setMaximum(info['max'])
                spin.setValue(step.get('value', info['default']))
                spin.valueChanged.connect(self._on_single_value_changed)
                layout.addWidget(spin)
                layout.addWidget(QLabel(info['unit']))
                self._spins = [('value', spin)]

            elif ctype == 'multi':
                values = step.get('value', {})
                for p in info['params']:
                    lbl = QLabel(f'{p["name"]}:')
                    lbl.setStyleSheet('font-size: 11px; color: #555;')
                    layout.addWidget(lbl)
                    spin = QSpinBox()
                    spin.setMinimum(p['min'])
                    spin.setMaximum(p['max'])
                    spin.setValue(values.get(p['name'], p['default']))
                    spin.setFixedWidth(64)
                    spin.valueChanged.connect(self._on_multi_value_changed)
                    layout.addWidget(spin)
                    self._spins.append((p['name'], spin))

            elif ctype == 'choice':
                combo = QComboBox()
                for ch in info['choices']:
                    combo.addItem(ch['label'], ch['value'])
                # Set current
                current_val = step.get('value', info['default'])
                for i, ch in enumerate(info['choices']):
                    if ch['value'] == current_val:
                        combo.setCurrentIndex(i)
                        break
                combo.currentIndexChanged.connect(self._on_choice_changed)
                layout.addWidget(combo)
                self._combo = combo

            elif ctype == 'none':
                layout.addWidget(QLabel('(instantáneo)'))

        layout.addStretch()

        # Botón eliminar (solo para pasos no fijos)
        if not step.get('fixed'):
            self.del_btn = QPushButton('✕')
            self.del_btn.setFixedSize(28, 28)
            self.del_btn.setStyleSheet(
                'QPushButton { color: #c00; font-weight: bold; }')
            self.del_btn.clicked.connect(lambda: self.delete_requested.emit(self))
            layout.addWidget(self.del_btn)

    # --- Callbacks de edición inline ---
    def _on_single_value_changed(self, v):
        self.step['value'] = v
        self.value_changed.emit()

    def _on_multi_value_changed(self):
        if not isinstance(self.step.get('value'), dict):
            self.step['value'] = {}
        for name, spin in self._spins:
            self.step['value'][name] = spin.value()
        self.value_changed.emit()

    def _on_choice_changed(self):
        if self._combo:
            self.step['value'] = self._combo.currentData()
            self.value_changed.emit()

    def update_index(self, new_index: int):
        self.num_label.setText(str(new_index))


# ====================================================================
# VENTANA PRINCIPAL
# ====================================================================
class MissionPlannerGUI(QMainWindow):
    def __init__(self, ros_node: MissionPlannerNode):
        super().__init__()
        self.ros_node = ros_node

        self.setWindowTitle('Mission Planner — DJI Tello')
        self.setMinimumSize(500, 750)

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(12, 12, 12, 12)
        root.setSpacing(10)

        # ---------------- Indicadores de estado ----------------
        status_bar = QHBoxLayout()
        self.status_label = QLabel('Estado: DISCONNECTED')
        self.battery_label = QLabel('Batería: --%')
        for w in (self.status_label, self.battery_label):
            w.setStyleSheet('font-weight: bold; padding: 4px 8px;')
        status_bar.addWidget(self.status_label)
        status_bar.addStretch()
        status_bar.addWidget(self.battery_label)
        root.addLayout(status_bar)

        # ---------------- Lista de pasos ----------------
        root.addWidget(QLabel('Secuencia de la misión:'))
        self.list_widget = QListWidget()
        self.list_widget.setDragDropMode(QAbstractItemView.InternalMove)
        self.list_widget.setDefaultDropAction(Qt.MoveAction)
        self.list_widget.model().rowsMoved.connect(self._on_rows_moved)
        root.addWidget(self.list_widget, stretch=1)

        # ---------------- Botón añadir paso ----------------
        add_btn = QPushButton('+ Añadir paso')
        add_btn.clicked.connect(self._on_add_step)
        root.addWidget(add_btn)

        # ---------------- Tiempo entre pasos ----------------
        time_layout = QHBoxLayout()
        time_layout.addWidget(QLabel('Tiempo entre pasos:'))
        self.wait_spin = QDoubleSpinBox()
        self.wait_spin.setMinimum(0.5)
        self.wait_spin.setMaximum(30.0)
        self.wait_spin.setSingleStep(0.5)
        self.wait_spin.setValue(4.0)
        self.wait_spin.setSuffix(' s')
        time_layout.addWidget(self.wait_spin)
        time_layout.addStretch()
        root.addLayout(time_layout)

        # ---------------- Umbral de batería crítica ----------------
        threshold_layout = QHBoxLayout()
        threshold_layout.addWidget(QLabel('Umbral batería crítica:'))
        self.threshold_spin = QSpinBox()
        self.threshold_spin.setMinimum(5)
        self.threshold_spin.setMaximum(95)
        self.threshold_spin.setSingleStep(1)
        self.threshold_spin.setValue(self.ros_node.battery_threshold)
        self.threshold_spin.setSuffix(' %')
        self.threshold_spin.valueChanged.connect(self._on_threshold_changed)
        threshold_layout.addWidget(self.threshold_spin)
        threshold_layout.addStretch()
        root.addLayout(threshold_layout)

        # ---------------- Botones de control ----------------
        ctrl_layout = QHBoxLayout()
        self.stop_btn = QPushButton('⬛  Stop')
        self.pause_btn = QPushButton('‖  Pausa')
        self.play_btn = QPushButton('▶  Play')
        for b in (self.stop_btn, self.pause_btn, self.play_btn):
            b.setMinimumHeight(44)
            b.setStyleSheet('font-size: 14px; font-weight: bold;')
        self.stop_btn.setStyleSheet(self.stop_btn.styleSheet() + ' background-color: #f5a8a8;')
        self.pause_btn.setStyleSheet(self.pause_btn.styleSheet() + ' background-color: #f5e1a8;')
        self.play_btn.setStyleSheet(self.play_btn.styleSheet() + ' background-color: #a8e6a8;')
        self.stop_btn.clicked.connect(self._on_stop)
        self.pause_btn.clicked.connect(self._on_pause)
        self.play_btn.clicked.connect(self._on_play)
        ctrl_layout.addWidget(self.stop_btn)
        ctrl_layout.addWidget(self.pause_btn)
        ctrl_layout.addWidget(self.play_btn)
        root.addLayout(ctrl_layout)

        # ---------------- Separador ----------------
        spacer = QWidget()
        spacer.setFixedHeight(0)
        root.addWidget(spacer)

        line = QFrame()
        line.setFrameShape(QFrame.HLine)
        line.setFrameShadow(QFrame.Sunken)
        root.addWidget(line)

        # ---------------- Log ----------------
        log_label = QLabel('Log:')
        log_label.setStyleSheet('margin-top: 4px;')
        root.addWidget(log_label)
        self.log_widget = QTextEdit()
        self.log_widget.setReadOnly(True)
        self.log_widget.setFont(QFont('Monospace', 9))
        self.log_widget.setStyleSheet(
            'background-color: #1e1e1e; color: #dcdcdc;')
        root.addWidget(self.log_widget, stretch=1)

        # ---------------- Conectar señales ----------------
        self.ros_node.log_signal.connect(self._append_log)
        self.ros_node.status_signal.connect(self._on_status_changed)
        self.ros_node.battery_signal.connect(self._on_battery_changed)
        self.ros_node.finished_signal.connect(self._on_finished)

        self._rebuild_list([dict(TAKEOFF_STEP), dict(LAND_STEP)])
        self._append_log('INFO', 'GUI inicializada.')

    # =================================================================
    # MANEJO DE LA LISTA
    # =================================================================
    def _rebuild_list(self, steps):
        self.list_widget.clear()
        for i, step in enumerate(steps, start=1):
            self._add_list_item(i, step)

    def _add_list_item(self, index, step):
        item = QListWidgetItem()
        widget = StepWidget(index, step)
        widget.delete_requested.connect(self._on_delete_step)
        if step.get('fixed'):
            item.setFlags(item.flags() & ~Qt.ItemIsDragEnabled)
        item.setSizeHint(widget.sizeHint())
        self.list_widget.addItem(item)
        self.list_widget.setItemWidget(item, widget)

    def _collect_steps(self):
        steps = []
        for i in range(self.list_widget.count()):
            item = self.list_widget.item(i)
            widget = self.list_widget.itemWidget(item)
            steps.append(widget.step)
        return steps

    def _renumber(self):
        for i in range(self.list_widget.count()):
            item = self.list_widget.item(i)
            widget = self.list_widget.itemWidget(item)
            widget.update_index(i + 1)

    def _on_rows_moved(self, *args):
        steps = self._collect_steps()
        if not steps or steps[0]['cmd'] != 'takeoff' or steps[-1]['cmd'] != 'land':
            QMessageBox.warning(self, 'Movimiento no permitido',
                'El despegue debe ser siempre el primer paso y el '
                'aterrizaje el último.')
            takeoff = next(s for s in steps if s['cmd'] == 'takeoff')
            land = next(s for s in steps if s['cmd'] == 'land')
            middle = [s for s in steps if s['cmd'] not in ('takeoff', 'land')]
            self._rebuild_list([takeoff] + middle + [land])
            return
        self._renumber()

    def _on_add_step(self):
        dialog = AddStepDialog(self)
        if dialog.exec_() != QDialog.Accepted:
            return
        new_step = dialog.get_step()
        if new_step is None:
            return
        steps = self._collect_steps()
        steps.insert(len(steps) - 1, new_step)
        self._rebuild_list(steps)

    def _on_delete_step(self, widget):
        for i in range(self.list_widget.count()):
            item = self.list_widget.item(i)
            if self.list_widget.itemWidget(item) is widget:
                self.list_widget.takeItem(i)
                break
        self._renumber()

    # =================================================================
    # BOTONES DE CONTROL
    # =================================================================
    def _on_play(self):
        if self.ros_node.mission_running and self.ros_node.mission_paused:
            self.ros_node.resume_mission()
            return
        if self.ros_node.mission_running:
            self._append_log('WARN', 'Ya hay una misión en curso.')
            return
        steps = self._collect_steps()
        if len(steps) < 2:
            QMessageBox.warning(self, 'Misión vacía',
                'La misión debe tener al menos despegue y aterrizaje.')
            return
        self.ros_node.start_mission(steps, self.wait_spin.value())
        self._set_controls_running(True)

    def _on_pause(self):
        self.ros_node.pause_mission()

    def _on_stop(self):
        self.ros_node.stop_mission()

    def _on_finished(self, reason):
        self._set_controls_running(False)

    def _on_threshold_changed(self, value):
        self.ros_node.battery_threshold = value

    def _set_controls_running(self, running):
        self.list_widget.setEnabled(not running)
        self.wait_spin.setEnabled(not running)
        self.threshold_spin.setEnabled(not running)

    # =================================================================
    # CALLBACKS DE SEÑALES
    # =================================================================
    def _append_log(self, level, message):
        ts = datetime.now().strftime('%H:%M:%S')
        color = {
            'INFO':  '#dcdcdc',
            'OK':    '#7fdc7f',
            'WARN':  '#dcc77f',
            'ERROR': '#dc7f7f',
            'CMD':   '#7fbcdc',
        }.get(level, '#dcdcdc')
        html = (f'<span style="color:#888">[{ts}]</span> '
                f'<span style="color:{color}">[{level}]</span> {message}')
        self.log_widget.append(html)

    def _on_status_changed(self, status):
        self.status_label.setText(f'Estado: {status}')
        color = {
            'DISCONNECTED': '#c00',
            'LANDED':       '#080',
            'FLYING':       '#06c',
        }.get(status, '#000')
        self.status_label.setStyleSheet(
            f'font-weight: bold; padding: 4px 8px; color: {color};')

    def _on_battery_changed(self, value):
        self.battery_label.setText(f'Batería: {value}%')
        if value <= self.ros_node.battery_threshold:
            color = '#c00'
        elif value < 50:
            color = '#c80'
        else:
            color = '#080'
        self.battery_label.setStyleSheet(
            f'font-weight: bold; padding: 4px 8px; color: {color};')


# ====================================================================
# MAIN
# ====================================================================
def ros_spin_thread(node):
    try:
        rclpy.spin(node)
    except Exception:
        pass


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()

    t = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    t.start()

    app = QApplication(sys.argv)
    gui = MissionPlannerGUI(node)
    gui.show()

    exit_code = app.exec_()

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == '__main__':
    main()