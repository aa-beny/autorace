import sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QSlider, QLabel, QWidget, QPushButton, QMessageBox, QFileDialog

import yaml

class ParameterAdjuster(Node):
    def __init__(self):
        super().__init__('parameter_adjuster')
        self.declare_parameter('top_x', 640.0)
        self.declare_parameter('top_y', 0.0)
        self.declare_parameter('bottom_x', 640.0)
        self.declare_parameter('bottom_y', 360.0)
        self.declare_parameter('hue_white_l', 0.0)
        self.declare_parameter('hue_white_h', 0.0)
        self.declare_parameter('saturation_white_l', 0.0)
        self.declare_parameter('saturation_white_h', 50.0)
        self.declare_parameter('lightness_white_l', 230.0)
        self.declare_parameter('lightness_white_h', 255.0)
        self.declare_parameter('reliability_white_line', 100.0)
        self.declare_parameter('hue_yellow_l', 8.0)
        self.declare_parameter('hue_yellow_h', 36.0)
        self.declare_parameter('saturation_yellow_l', 8.0)
        self.declare_parameter('saturation_yellow_h', 80.0)
        self.declare_parameter('lightness_yellow_l', 240.0)
        self.declare_parameter('lightness_yellow_h', 255.0)
        self.declare_parameter('reliability_yellow_line', 100.0)
        self.get_logger().info('ParameterAdjuster node initialized')
        self.publisher_top_x = self.create_publisher(Float64, '/detect/parameter/top_x', QoSProfile(depth=10))
        self.publisher_top_y = self.create_publisher(Float64, '/detect/parameter/top_y', QoSProfile(depth=10))
        self.publisher_bottom_x = self.create_publisher(Float64, '/detect/parameter/bottom_x', QoSProfile(depth=10))
        self.publisher_bottom_y = self.create_publisher(Float64, '/detect/parameter/bottom_y', QoSProfile(depth=10))
        self.publisher_hue_white_l = self.create_publisher(Float64, '/detect/parameter/hue_white_l', QoSProfile(depth=10))
        self.publisher_hue_white_h = self.create_publisher(Float64, '/detect/parameter/hue_white_h', QoSProfile(depth=10))
        self.publisher_saturation_white_l = self.create_publisher(Float64, '/detect/parameter/saturation_white_l', QoSProfile(depth=10))
        self.publisher_saturation_white_h = self.create_publisher(Float64, '/detect/parameter/saturation_white_h', QoSProfile(depth=10))
        self.publisher_lightness_white_l = self.create_publisher(Float64, '/detect/parameter/lightness_white_l', QoSProfile(depth=10))
        self.publisher_lightness_white_h = self.create_publisher(Float64, '/detect/parameter/lightness_white_h', QoSProfile(depth=10))
        self.publisher_reliability_white_line = self.create_publisher(Float64, '/detect/parameter/reliability_white_line', QoSProfile(depth=10))
        self.publisher_hue_yellow_l = self.create_publisher(Float64, '/detect/parameter/hue_yellow_l', QoSProfile(depth=10))
        self.publisher_hue_yellow_h = self.create_publisher(Float64, '/detect/parameter/hue_yellow_h', QoSProfile(depth=10))
        self.publisher_saturation_yellow_l = self.create_publisher(Float64, '/detect/parameter/saturation_yellow_l', QoSProfile(depth=10))
        self.publisher_saturation_yellow_h = self.create_publisher(Float64, '/detect/parameter/saturation_yellow_h', QoSProfile(depth=10))
        self.publisher_lightness_yellow_l = self.create_publisher(Float64, '/detect/parameter/lightness_yellow_l', QoSProfile(depth=10))
        self.publisher_lightness_yellow_h = self.create_publisher(Float64, '/detect/parameter/lightness_yellow_h', QoSProfile(depth=10))
        self.publisher_reliability_yellow_line = self.create_publisher(Float64, '/detect/parameter/reliability_yellow_line', QoSProfile(depth=10))

    def adjust_parameters(self, param_name, value):
        param_value = Parameter(param_name, Parameter.Type.DOUBLE, value)
        self.set_parameters([param_value])
        self.get_logger().info(f'Parameter adjusted: {param_name} - {value}')
        msg = Float64()
        msg.data = value
        topic_name = f'{param_name}_topic'
        publisher = getattr(self, f'publisher_{param_name}')
        publisher.publish(msg)

    def save_parameters_to_yaml(self):
        path = "/".join(sys.path[0].split("/")[1:4])

        node_name = '/detect_node'
        parameters = {
            'ros__parameters': {}
        }
        parameter_names = [
            'top_x', 'top_y', 'bottom_x', 'bottom_y', 'hue_white_l', 'hue_white_h',
            'saturation_white_l', 'saturation_white_h', 'lightness_white_l', 'lightness_white_h',
            'reliability_white_line', 'hue_yellow_l', 'hue_yellow_h', 'saturation_yellow_l',
            'saturation_yellow_h', 'lightness_yellow_l', 'lightness_yellow_h', 'reliability_yellow_line'
        ]
        for param_name in parameter_names:
            param_value = self.get_parameter(param_name)
            parameters['ros__parameters'][param_name] = param_value.value
        with open('/' + path + '/src/detect/detect_lane/config/' + 'hsv_parameters_own.yaml', 'w') as yaml_file:
            yaml.dump({node_name: parameters}, yaml_file)
        self.get_logger().info('Parameters saved to parameters.yaml')

class SliderWindow(QMainWindow):
    def __init__(self, adjuster_node):
        super().__init__()
        self.adjuster_node = adjuster_node
        self.setWindowTitle('Parameter Slider')
        self.setGeometry(100, 100, 400, 250)

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        self.layout = QVBoxLayout()
        self.central_widget.setLayout(self.layout)

        self.slider_top_x = QSlider()
        self.slider_top_x.setMinimum(0)
        self.slider_top_x.setMaximum(640)
        self.slider_top_x.setValue(640)
        self.slider_top_x.setOrientation(1)  # Vertical orientation

        self.label_top_x = QLabel('Top X Value: 640')

        self.slider_top_y = QSlider()
        self.slider_top_y.setMinimum(0)
        self.slider_top_y.setMaximum(480)
        self.slider_top_y.setValue(0)
        self.slider_top_y.setOrientation(1)  # Vertical orientation

        self.label_top_y = QLabel('Top Y Value: 0')

        self.slider_bottom_x = QSlider()
        self.slider_bottom_x.setMinimum(0)
        self.slider_bottom_x.setMaximum(640)
        self.slider_bottom_x.setValue(640)
        self.slider_bottom_x.setOrientation(1)  # Vertical orientation

        self.label_bottom_x = QLabel('Bottom X Value: 640')

        self.slider_bottom_y = QSlider()
        self.slider_bottom_y.setMinimum(0)
        self.slider_bottom_y.setMaximum(480)
        self.slider_bottom_y.setValue(360)
        self.slider_bottom_y.setOrientation(1)  # Vertical orientation

        self.label_bottom_y = QLabel('Bottom Y Value: 360')
        
        self.slider_hue_white_l = QSlider()
        self.slider_hue_white_l.setMinimum(0)
        self.slider_hue_white_l.setMaximum(255)
        self.slider_hue_white_l.setValue(0)
        self.slider_hue_white_l.setOrientation(1)  # Vertical orientation

        self.label_hue_white_l = QLabel('hue_white_l Value: 0')

        self.slider_hue_white_h = QSlider()
        self.slider_hue_white_h.setMinimum(0)
        self.slider_hue_white_h.setMaximum(255)
        self.slider_hue_white_h.setValue(0)
        self.slider_hue_white_h.setOrientation(1)  # Vertical orientation

        self.label_hue_white_h = QLabel('hue_white_h Value: 0')

        self.slider_saturation_white_l = QSlider()
        self.slider_saturation_white_l.setMinimum(0)
        self.slider_saturation_white_l.setMaximum(255)
        self.slider_saturation_white_l.setValue(0)
        self.slider_saturation_white_l.setOrientation(1)  # Vertical orientation

        self.label_saturation_white_l = QLabel('saturation_white_l Value: 0')

        self.slider_saturation_white_h = QSlider()
        self.slider_saturation_white_h.setMinimum(0)
        self.slider_saturation_white_h.setMaximum(255)
        self.slider_saturation_white_h.setValue(50)
        self.slider_saturation_white_h.setOrientation(1)  # Vertical orientation

        self.label_saturation_white_h = QLabel('saturation_white_h Value: 50')

        self.slider_lightness_white_l = QSlider()
        self.slider_lightness_white_l.setMinimum(0)
        self.slider_lightness_white_l.setMaximum(255)
        self.slider_lightness_white_l.setValue(230)
        self.slider_lightness_white_l.setOrientation(1)  # Vertical orientation

        self.label_lightness_white_l = QLabel('lightness_white_l Value: 230')

        self.slider_lightness_white_h = QSlider()
        self.slider_lightness_white_h.setMinimum(0)
        self.slider_lightness_white_h.setMaximum(255)
        self.slider_lightness_white_h.setValue(255)
        self.slider_lightness_white_h.setOrientation(1)  # Vertical orientation

        self.label_lightness_white_h = QLabel('lightness_white_h Value: 255')

        self.slider_reliability_white_line = QSlider()
        self.slider_reliability_white_line.setMinimum(0)
        self.slider_reliability_white_line.setMaximum(100)
        self.slider_reliability_white_line.setValue(100)
        self.slider_reliability_white_line.setOrientation(1)  # Vertical orientation

        self.label_reliability_white_line = QLabel('reliability_white_line Value: 100')

        #===================
        self.slider_hue_yellow_l = QSlider()
        self.slider_hue_yellow_l.setMinimum(0)
        self.slider_hue_yellow_l.setMaximum(255)
        self.slider_hue_yellow_l.setValue(8)
        self.slider_hue_yellow_l.setOrientation(1)  # Vertical orientation

        self.label_hue_yellow_l = QLabel('hue_yellow_l Value: 8')

        self.slider_hue_yellow_h = QSlider()
        self.slider_hue_yellow_h.setMinimum(0)
        self.slider_hue_yellow_h.setMaximum(255)
        self.slider_hue_yellow_h.setValue(36)
        self.slider_hue_yellow_h.setOrientation(1)  # Vertical orientation

        self.label_hue_yellow_h = QLabel('hue_yellow_h Value: 36')

        self.slider_saturation_yellow_l = QSlider()
        self.slider_saturation_yellow_l.setMinimum(0)
        self.slider_saturation_yellow_l.setMaximum(255)
        self.slider_saturation_yellow_l.setValue(8)
        self.slider_saturation_yellow_l.setOrientation(1)  # Vertical orientation

        self.label_saturation_yellow_l = QLabel('saturation_yellow_l Value: 8')

        self.slider_saturation_yellow_h = QSlider()
        self.slider_saturation_yellow_h.setMinimum(0)
        self.slider_saturation_yellow_h.setMaximum(255)
        self.slider_saturation_yellow_h.setValue(80)
        self.slider_saturation_yellow_h.setOrientation(1)  # Vertical orientation

        self.label_saturation_yellow_h = QLabel('saturation_yellow_h Value: 80')

        self.slider_lightness_yellow_l = QSlider()
        self.slider_lightness_yellow_l.setMinimum(0)
        self.slider_lightness_yellow_l.setMaximum(255)
        self.slider_lightness_yellow_l.setValue(240)
        self.slider_lightness_yellow_l.setOrientation(1)  # Vertical orientation

        self.label_lightness_yellow_l = QLabel('lightness_yellow_l Value: 240')

        self.slider_lightness_yellow_h = QSlider()
        self.slider_lightness_yellow_h.setMinimum(0)
        self.slider_lightness_yellow_h.setMaximum(255)
        self.slider_lightness_yellow_h.setValue(255)
        self.slider_lightness_yellow_h.setOrientation(1)  # Vertical orientation

        self.label_lightness_yellow_h = QLabel('lightness_yellow_h Value: 255')

        self.slider_reliability_yellow_line = QSlider()
        self.slider_reliability_yellow_line.setMinimum(0)
        self.slider_reliability_yellow_line.setMaximum(100)
        self.slider_reliability_yellow_line.setValue(100)
        self.slider_reliability_yellow_line.setOrientation(1)  # Vertical orientation

        self.label_reliability_yellow_line = QLabel('reliability_yellow_line Value: 100')

        self.save_button = QPushButton('Save Parameters')

        #===================
        self.open_file_button = QPushButton('Open File')
        self.layout.addWidget(self.open_file_button)
        self.open_file_button.clicked.connect(self.open_file_dialog)
        #===================

        self.layout.addWidget(self.label_top_x)
        self.layout.addWidget(self.slider_top_x)
        self.layout.addWidget(self.label_top_y)
        self.layout.addWidget(self.slider_top_y)
        self.layout.addWidget(self.label_bottom_x)
        self.layout.addWidget(self.slider_bottom_x)
        self.layout.addWidget(self.label_bottom_y)
        self.layout.addWidget(self.slider_bottom_y)
        self.layout.addWidget(self.label_hue_white_l)
        self.layout.addWidget(self.slider_hue_white_l)
        self.layout.addWidget(self.label_hue_white_h)
        self.layout.addWidget(self.slider_hue_white_h)
        self.layout.addWidget(self.label_saturation_white_l)
        self.layout.addWidget(self.slider_saturation_white_l)
        self.layout.addWidget(self.label_saturation_white_h)
        self.layout.addWidget(self.slider_saturation_white_h)
        self.layout.addWidget(self.label_lightness_white_l)
        self.layout.addWidget(self.slider_lightness_white_l)
        self.layout.addWidget(self.label_lightness_white_h)
        self.layout.addWidget(self.slider_lightness_white_h)
        self.layout.addWidget(self.label_reliability_white_line)
        self.layout.addWidget(self.slider_reliability_white_line)

        self.layout.addWidget(self.label_hue_yellow_l)
        self.layout.addWidget(self.slider_hue_yellow_l)
        self.layout.addWidget(self.label_hue_yellow_h)
        self.layout.addWidget(self.slider_hue_yellow_h)
        self.layout.addWidget(self.label_saturation_yellow_l)
        self.layout.addWidget(self.slider_saturation_yellow_l)
        self.layout.addWidget(self.label_saturation_yellow_h)
        self.layout.addWidget(self.slider_saturation_yellow_h)
        self.layout.addWidget(self.label_lightness_yellow_l)
        self.layout.addWidget(self.slider_lightness_yellow_l)
        self.layout.addWidget(self.label_lightness_yellow_h)
        self.layout.addWidget(self.slider_lightness_yellow_h)
        self.layout.addWidget(self.label_reliability_yellow_line)
        self.layout.addWidget(self.slider_reliability_yellow_line)
        self.layout.addWidget(self.save_button)

        self.slider_top_x.valueChanged.connect(self.slider_top_x_changed)
        self.slider_top_y.valueChanged.connect(self.slider_top_y_changed)
        self.slider_bottom_x.valueChanged.connect(self.slider_bottom_x_changed)
        self.slider_bottom_y.valueChanged.connect(self.slider_bottom_y_changed)
        self.slider_hue_white_l.valueChanged.connect(self.slider_hue_white_l_changed)
        self.slider_hue_white_h.valueChanged.connect(self.slider_hue_white_h_changed)
        self.slider_saturation_white_l.valueChanged.connect(self.slider_saturation_white_l_changed)
        self.slider_saturation_white_h.valueChanged.connect(self.slider_saturation_white_h_changed)
        self.slider_lightness_white_l.valueChanged.connect(self.slider_lightness_white_l_changed)
        self.slider_lightness_white_h.valueChanged.connect(self.slider_lightness_white_h_changed)
        self.slider_reliability_white_line.valueChanged.connect(self.slider_reliability_white_line_changed)

        self.slider_hue_yellow_l.valueChanged.connect(self.slider_hue_yellow_l_changed)
        self.slider_hue_yellow_h.valueChanged.connect(self.slider_hue_yellow_h_changed)
        self.slider_saturation_yellow_l.valueChanged.connect(self.slider_saturation_yellow_l_changed)
        self.slider_saturation_yellow_h.valueChanged.connect(self.slider_saturation_yellow_h_changed)
        self.slider_lightness_yellow_l.valueChanged.connect(self.slider_lightness_yellow_l_changed)
        self.slider_lightness_yellow_h.valueChanged.connect(self.slider_lightness_yellow_h_changed)
        self.slider_reliability_yellow_line.valueChanged.connect(self.slider_reliability_yellow_line_changed)
        self.save_button.clicked.connect(self.save_button_clicked)

    def slider_top_x_changed(self, value):
        parameter_value = float(value)
        self.label_top_x.setText(f'Top X Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('top_x', parameter_value)

    def slider_top_y_changed(self, value):
        parameter_value = float(value)
        self.label_top_y.setText(f'Top Y Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('top_y', parameter_value)

    def slider_bottom_x_changed(self, value):
        parameter_value = float(value)
        self.label_bottom_x.setText(f'Bottom X Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('bottom_x', parameter_value)

    def slider_bottom_y_changed(self, value):
        parameter_value = float(value)
        self.label_bottom_y.setText(f'Bottom Y Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('bottom_y', parameter_value)

    def slider_hue_white_l_changed(self, value):
        parameter_value = float(value)
        self.label_hue_white_l.setText(f'hue_white_l Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('hue_white_l', parameter_value)

    def slider_hue_white_h_changed(self, value):
        parameter_value = float(value)
        self.label_hue_white_h.setText(f'hue_white_h Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('hue_white_h', parameter_value)

    def slider_saturation_white_l_changed(self, value):
        parameter_value = float(value)
        self.label_saturation_white_l.setText(f'saturation_white_l Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('saturation_white_l', parameter_value)

    def slider_saturation_white_h_changed(self, value):
        parameter_value = float(value)
        self.label_saturation_white_h.setText(f'saturation_white_h Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('saturation_white_h', parameter_value)

    def slider_lightness_white_l_changed(self, value):
        parameter_value = float(value)
        self.label_lightness_white_l.setText(f'lightness_white_l Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('lightness_white_l', parameter_value)

    def slider_lightness_white_h_changed(self, value):
        parameter_value = float(value)
        self.label_lightness_white_h.setText(f'lightness_white_h Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('lightness_white_h', parameter_value)

    def slider_reliability_white_line_changed(self, value):
        parameter_value = float(value)
        self.label_lightness_white_h.setText(f'reliability_white_line Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('reliability_white_line', parameter_value)

    def slider_hue_yellow_l_changed(self, value):
        parameter_value = float(value)
        self.label_hue_yellow_l.setText(f'hue_yellow_l Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('hue_yellow_l', parameter_value)

    def slider_hue_yellow_h_changed(self, value):
        parameter_value = float(value)
        self.label_hue_yellow_h.setText(f'hue_yellow_h Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('hue_yellow_h', parameter_value)

    def slider_saturation_yellow_l_changed(self, value):
        parameter_value = float(value)
        self.label_saturation_yellow_l.setText(f'saturation_yellow_l Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('saturation_yellow_l', parameter_value)

    def slider_saturation_yellow_h_changed(self, value):
        parameter_value = float(value)
        self.label_saturation_yellow_h.setText(f'saturation_yellow_h Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('saturation_yellow_h', parameter_value)

    def slider_lightness_yellow_l_changed(self, value):
        parameter_value = float(value)
        self.label_lightness_yellow_l.setText(f'lightness_yellow_l Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('lightness_yellow_l', parameter_value)

    def slider_lightness_yellow_h_changed(self, value):
        parameter_value = float(value)
        self.label_lightness_yellow_h.setText(f'lightness_yellow_h Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('lightness_yellow_h', parameter_value)

    def slider_reliability_yellow_line_changed(self, value):
        parameter_value = float(value)
        self.label_reliability_yellow_line.setText(f'reliability_yellow_line Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('reliability_yellow_line', parameter_value)

    def save_button_clicked(self):
        self.adjuster_node.save_parameters_to_yaml()
        QMessageBox.information(self, 'Saved', 'Parameters saved to parameters.yaml')

    def open_file_dialog(self):
        options = QFileDialog.Options()
        file_name, _ = QFileDialog.getOpenFileName(self, "Open File", "", "YAML Files (*.yaml);;All Files (*)", options=options)
        if file_name:
            self.load_parameters_from_yaml(file_name)

    def load_parameters_from_yaml(self, file_name):
        with open(file_name, 'r') as yaml_file:
            yaml_data = yaml.load(yaml_file, Loader=yaml.FullLoader)
            if yaml_data and '/detect_node' in yaml_data and 'ros__parameters' in yaml_data['/detect_node']:
                parameters = yaml_data['/detect_node']['ros__parameters']
                self.update_gui_with_parameters(parameters)
            else:
                QMessageBox.warning(self, 'Error', 'Invalid YAML file format')

    def update_gui_with_parameters(self, parameters):
        self.slider_top_x.setValue(int(parameters.get('top_x', 640)))
        self.slider_top_y.setValue(int(parameters.get('top_y', 0)))
        self.slider_bottom_x.setValue(int(parameters.get('bottom_x', 640)))
        self.slider_bottom_y.setValue(int(parameters.get('bottom_y', 360)))
        self.slider_hue_white_l.setValue(int(parameters.get('hue_white_l', 0)))
        self.slider_hue_white_h.setValue(int(parameters.get('hue_white_h', 0)))
        self.slider_saturation_white_l.setValue(int(parameters.get('saturation_white_l', 0)))
        self.slider_saturation_white_h.setValue(int(parameters.get('saturation_white_h', 50)))
        self.slider_lightness_white_l.setValue(int(parameters.get('lightness_white_l', 230)))
        self.slider_lightness_white_h.setValue(int(parameters.get('lightness_white_h', 255)))
        self.slider_reliability_white_line.setValue(int(parameters.get('reliability_white_line', 100)))
        self.slider_hue_yellow_l.setValue(int(parameters.get('hue_yellow_l', 8)))
        self.slider_hue_yellow_h.setValue(int(parameters.get('hue_yellow_h', 36)))
        self.slider_saturation_yellow_l.setValue(int(parameters.get('saturation_yellow_l', 8)))
        self.slider_saturation_yellow_h.setValue(int(parameters.get('saturation_yellow_h', 80)))
        self.slider_lightness_yellow_l.setValue(int(parameters.get('lightness_yellow_l', 240)))
        self.slider_lightness_yellow_h.setValue(int(parameters.get('lightness_yellow_h', 255)))
        self.slider_reliability_yellow_line.setValue(int(parameters.get('reliability_yellow_line', 100)))

        # Update labels
        self.label_top_x.setText(f'Top X Value: {parameters.get("top_x", 640)}')
        self.label_top_y.setText(f'Top Y Value: {parameters.get("top_y", 0)}')
        self.label_bottom_x.setText(f'Bottom X Value: {parameters.get("bottom_x", 640)}')
        self.label_bottom_y.setText(f'Bottom Y Value: {parameters.get("bottom_y", 360)}')
        self.label_hue_white_l.setText(f'hue_white_l Value: {parameters.get("hue_white_l", 0)}')
        self.label_hue_white_h.setText(f'hue_white_h Value: {parameters.get("hue_white_h", 0)}')
        self.label_saturation_white_l.setText(f'saturation_white_l Value: {parameters.get("saturation_white_l", 0)}')
        self.label_saturation_white_h.setText(f'saturation_white_h Value: {parameters.get("saturation_white_h", 50)}')
        self.label_lightness_white_l.setText(f'lightness_white_l Value: {parameters.get("lightness_white_l", 230)}')
        self.label_lightness_white_h.setText(f'lightness_white_h Value: {parameters.get("lightness_white_h", 255)}')
        self.label_reliability_white_line.setText(f'reliability_white_line Value: {parameters.get("reliability_white_line", 100)}')
        self.label_hue_yellow_l.setText(f'hue_yellow_l Value: {parameters.get("hue_yellow_l", 8)}')
        self.label_hue_yellow_h.setText(f'hue_yellow_h Value: {parameters.get("hue_yellow_h", 36)}')
        self.label_saturation_yellow_l.setText(f'saturation_yellow_l Value: {parameters.get("saturation_yellow_l", 8)}')
        self.label_saturation_yellow_h.setText(f'saturation_yellow_h Value: {parameters.get("saturation_yellow_h", 80)}')
        self.label_lightness_yellow_l.setText(f'lightness_yellow_l Value: {parameters.get("lightness_yellow_l", 240)}')
        self.label_lightness_yellow_h.setText(f'lightness_yellow_h Value: {parameters.get("lightness_yellow_h", 255)}')
        self.label_reliability_yellow_line.setText(f'reliability_yellow_line Value: {parameters.get("reliability_yellow_line", 100)}')

def main():
    rclpy.init()
    adjuster = ParameterAdjuster()
    app = QApplication(sys.argv)
    window = SliderWindow(adjuster)
    window.show()
    
    try:
        sys.exit(app.exec_())
    except Exception as e:
        print(e)
    finally:
        QApplication.quit()
        adjuster.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
