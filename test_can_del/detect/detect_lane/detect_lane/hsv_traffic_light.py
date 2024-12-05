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

        self.declare_parameter('boundcenter_x', 480.0)
        self.declare_parameter('boundcenter_y', 320.0)
        self.declare_parameter('boundwidth', 50.0)
        self.declare_parameter('boundheight', 100.0)

        self.declare_parameter('hue_red_l', 0.0)
        self.declare_parameter('hue_red_h', 0.0)
        self.declare_parameter('saturation_red_l', 0.0)
        self.declare_parameter('saturation_red_h', 50.0)
        self.declare_parameter('lightness_red_l', 230.0)
        self.declare_parameter('lightness_red_h', 255.0)

        self.declare_parameter('hue_green_l', 8.0)
        self.declare_parameter('hue_green_h', 36.0)
        self.declare_parameter('saturation_green_l', 8.0)
        self.declare_parameter('saturation_green_h', 80.0)
        self.declare_parameter('lightness_green_l', 240.0)
        self.declare_parameter('lightness_green_h', 255.0)

        self.declare_parameter('boundsize', 50.0) 

        self.get_logger().info('ParameterAdjuster node initialized')
        self.publisher_boundcenter_x = self.create_publisher(Float64, '/detect/parameter/boundcenter_x', QoSProfile(depth=10))
        self.publisher_boundcenter_y = self.create_publisher(Float64, '/detect/parameter/boundcenter_y', QoSProfile(depth=10))
        self.publisher_boundwidth = self.create_publisher(Float64, '/detect/parameter/boundwidth', QoSProfile(depth=10))
        self.publisher_boundheight = self.create_publisher(Float64, '/detect/parameter/boundheight', QoSProfile(depth=10))

        self.publisher_hue_red_l = self.create_publisher(Float64, '/detect/parameter/hue_red_l', QoSProfile(depth=10))
        self.publisher_hue_red_h = self.create_publisher(Float64, '/detect/parameter/hue_red_h', QoSProfile(depth=10))
        self.publisher_saturation_red_l = self.create_publisher(Float64, '/detect/parameter/saturation_red_l', QoSProfile(depth=10))
        self.publisher_saturation_red_h = self.create_publisher(Float64, '/detect/parameter/saturation_red_h', QoSProfile(depth=10))
        self.publisher_lightness_red_l = self.create_publisher(Float64, '/detect/parameter/lightness_red_l', QoSProfile(depth=10))
        self.publisher_lightness_red_h = self.create_publisher(Float64, '/detect/parameter/lightness_red_h', QoSProfile(depth=10))

        self.publisher_hue_green_l = self.create_publisher(Float64, '/detect/parameter/hue_green_l', QoSProfile(depth=10))
        self.publisher_hue_green_h = self.create_publisher(Float64, '/detect/parameter/hue_green_h', QoSProfile(depth=10))
        self.publisher_saturation_green_l = self.create_publisher(Float64, '/detect/parameter/saturation_green_l', QoSProfile(depth=10))
        self.publisher_saturation_green_h = self.create_publisher(Float64, '/detect/parameter/saturation_green_h', QoSProfile(depth=10))
        self.publisher_lightness_green_l = self.create_publisher(Float64, '/detect/parameter/lightness_green_l', QoSProfile(depth=10))
        self.publisher_lightness_green_h = self.create_publisher(Float64, '/detect/parameter/lightness_green_h', QoSProfile(depth=10))

        self.publisher_boundsize = self.create_publisher(Float64, '/detect/parameter/boundsize', QoSProfile(depth=10))

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
            'hue_red_l', 'hue_red_h',
            'saturation_red_l', 'saturation_red_h', 'lightness_red_l', 'lightness_red_h',
            'hue_green_l', 'hue_green_h', 'saturation_green_l',
            'saturation_green_h', 'lightness_green_l', 'lightness_green_h', 'boundsize',
            'boundcenter_x', 'boundcenter_y', 'boundwidth', 'boundheight'
        ]
        for param_name in parameter_names:
            param_value = self.get_parameter(param_name)
            parameters['ros__parameters'][param_name] = param_value.value
        with open('/' + path + '/src/detect/detect_lane/config/' + 'hsv_parameters_tr_own.yaml', 'w') as yaml_file:
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

        self.slider_boundcenter_x = QSlider()
        self.slider_boundcenter_x.setMinimum(0) 
        self.slider_boundcenter_x.setMaximum(640)
        self.slider_boundcenter_x.setValue(480)
        self.slider_boundcenter_x.setOrientation(1)  # Vertical orientation

        self.label_boundcenter_x = QLabel('boundcenter_x Value: 480')

        self.slider_boundcenter_y = QSlider()
        self.slider_boundcenter_y.setMinimum(0)
        self.slider_boundcenter_y.setMaximum(480)
        self.slider_boundcenter_y.setValue(320)
        self.slider_boundcenter_y.setOrientation(1)  # Vertical orientation

        self.label_boundcenter_y = QLabel('boundcenter_y Value: 320')

        self.slider_boundwidth = QSlider()
        self.slider_boundwidth.setMinimum(0)
        self.slider_boundwidth.setMaximum(640)
        self.slider_boundwidth.setValue(50)
        self.slider_boundwidth.setOrientation(1)  # Vertical orientation

        self.label_boundwidth = QLabel('boundwidth Value: 50')

        self.slider_boundheight = QSlider()
        self.slider_boundheight.setMinimum(0)
        self.slider_boundheight.setMaximum(480)
        self.slider_boundheight.setValue(100)
        self.slider_boundheight.setOrientation(1)  # Vertical orientation

        self.label_boundheight = QLabel('boundheight Value: 100')
        
        self.slider_hue_red_l = QSlider()
        self.slider_hue_red_l.setMinimum(0)
        self.slider_hue_red_l.setMaximum(179)
        self.slider_hue_red_l.setValue(0)
        self.slider_hue_red_l.setOrientation(1)  # Vertical orientation

        self.label_hue_red_l = QLabel('hue_red_l Value: 0')

        self.slider_hue_red_h = QSlider()
        self.slider_hue_red_h.setMinimum(0)
        self.slider_hue_red_h.setMaximum(179)
        self.slider_hue_red_h.setValue(0)
        self.slider_hue_red_h.setOrientation(1)  # Vertical orientation

        self.label_hue_red_h = QLabel('hue_red_h Value: 0')

        self.slider_saturation_red_l = QSlider()
        self.slider_saturation_red_l.setMinimum(0)
        self.slider_saturation_red_l.setMaximum(255)
        self.slider_saturation_red_l.setValue(0)
        self.slider_saturation_red_l.setOrientation(1)  # Vertical orientation

        self.label_saturation_red_l = QLabel('saturation_red_l Value: 0')

        self.slider_saturation_red_h = QSlider()
        self.slider_saturation_red_h.setMinimum(0)
        self.slider_saturation_red_h.setMaximum(255)
        self.slider_saturation_red_h.setValue(50)
        self.slider_saturation_red_h.setOrientation(1)  # Vertical orientation

        self.label_saturation_red_h = QLabel('saturation_red_h Value: 50')

        self.slider_lightness_red_l = QSlider()
        self.slider_lightness_red_l.setMinimum(0)
        self.slider_lightness_red_l.setMaximum(255)
        self.slider_lightness_red_l.setValue(230)
        self.slider_lightness_red_l.setOrientation(1)  # Vertical orientation

        self.label_lightness_red_l = QLabel('lightness_red_l Value: 230')

        self.slider_lightness_red_h = QSlider()
        self.slider_lightness_red_h.setMinimum(0)
        self.slider_lightness_red_h.setMaximum(255)
        self.slider_lightness_red_h.setValue(255)
        self.slider_lightness_red_h.setOrientation(1)  # Vertical orientation

        self.label_lightness_red_h = QLabel('lightness_red_h Value: 255')

        #===================
        self.slider_hue_green_l = QSlider()
        self.slider_hue_green_l.setMinimum(0)
        self.slider_hue_green_l.setMaximum(255)
        self.slider_hue_green_l.setValue(8)
        self.slider_hue_green_l.setOrientation(1)  # Vertical orientation

        self.label_hue_green_l = QLabel('hue_green_l Value: 8')

        self.slider_hue_green_h = QSlider()
        self.slider_hue_green_h.setMinimum(0)
        self.slider_hue_green_h.setMaximum(255)
        self.slider_hue_green_h.setValue(36)
        self.slider_hue_green_h.setOrientation(1)  # Vertical orientation

        self.label_hue_green_h = QLabel('hue_green_h Value: 36')

        self.slider_saturation_green_l = QSlider()
        self.slider_saturation_green_l.setMinimum(0)
        self.slider_saturation_green_l.setMaximum(255)
        self.slider_saturation_green_l.setValue(8)
        self.slider_saturation_green_l.setOrientation(1)  # Vertical orientation

        self.label_saturation_green_l = QLabel('saturation_green_l Value: 8')

        self.slider_saturation_green_h = QSlider()
        self.slider_saturation_green_h.setMinimum(0)
        self.slider_saturation_green_h.setMaximum(255)
        self.slider_saturation_green_h.setValue(80)
        self.slider_saturation_green_h.setOrientation(1)  # Vertical orientation

        self.label_saturation_green_h = QLabel('saturation_green_h Value: 80')

        self.slider_lightness_green_l = QSlider()
        self.slider_lightness_green_l.setMinimum(0)
        self.slider_lightness_green_l.setMaximum(255)
        self.slider_lightness_green_l.setValue(240)
        self.slider_lightness_green_l.setOrientation(1)  # Vertical orientation

        self.label_lightness_green_l = QLabel('lightness_green_l Value: 240')

        self.slider_lightness_green_h = QSlider()
        self.slider_lightness_green_h.setMinimum(0)
        self.slider_lightness_green_h.setMaximum(255)
        self.slider_lightness_green_h.setValue(255)
        self.slider_lightness_green_h.setOrientation(1)  # Vertical orientation

        self.label_lightness_green_h = QLabel('lightness_green_h Value: 255')

        self.slider_boundsize = QSlider()
        self.slider_boundsize.setMinimum(0)
        self.slider_boundsize.setMaximum(100)
        self.slider_boundsize.setValue(50)
        self.slider_boundsize.setOrientation(1)  # Vertical orientation

        self.label_boundsize = QLabel('boundsize Value: 50')

        self.save_button = QPushButton('Save Parameters')

        #===================
        self.open_file_button = QPushButton('Open File')
        self.layout.addWidget(self.open_file_button)
        self.open_file_button.clicked.connect(self.open_file_dialog)
        #===================
        
        self.layout.addWidget(self.label_boundcenter_x)
        self.layout.addWidget(self.slider_boundcenter_x)
        self.layout.addWidget(self.label_boundcenter_y)
        self.layout.addWidget(self.slider_boundcenter_y)
        self.layout.addWidget(self.label_boundwidth)
        self.layout.addWidget(self.slider_boundwidth)
        self.layout.addWidget(self.label_boundheight)
        self.layout.addWidget(self.slider_boundheight)

        self.layout.addWidget(self.label_hue_red_l)
        self.layout.addWidget(self.slider_hue_red_l)
        self.layout.addWidget(self.label_hue_red_h)
        self.layout.addWidget(self.slider_hue_red_h)
        self.layout.addWidget(self.label_saturation_red_l)
        self.layout.addWidget(self.slider_saturation_red_l)
        self.layout.addWidget(self.label_saturation_red_h)
        self.layout.addWidget(self.slider_saturation_red_h)
        self.layout.addWidget(self.label_lightness_red_l)
        self.layout.addWidget(self.slider_lightness_red_l)
        self.layout.addWidget(self.label_lightness_red_h)
        self.layout.addWidget(self.slider_lightness_red_h)

        self.layout.addWidget(self.label_hue_green_l)
        self.layout.addWidget(self.slider_hue_green_l)
        self.layout.addWidget(self.label_hue_green_h)
        self.layout.addWidget(self.slider_hue_green_h)
        self.layout.addWidget(self.label_saturation_green_l)
        self.layout.addWidget(self.slider_saturation_green_l)
        self.layout.addWidget(self.label_saturation_green_h)
        self.layout.addWidget(self.slider_saturation_green_h)
        self.layout.addWidget(self.label_lightness_green_l)
        self.layout.addWidget(self.slider_lightness_green_l)
        self.layout.addWidget(self.label_lightness_green_h)
        self.layout.addWidget(self.slider_lightness_green_h)
        
        self.layout.addWidget(self.label_boundsize)
        self.layout.addWidget(self.slider_boundsize)

        self.layout.addWidget(self.save_button)

        self.slider_boundcenter_x.valueChanged.connect(self.slider_boundcenter_x_changed)
        self.slider_boundcenter_y.valueChanged.connect(self.slider_boundcenter_y_changed)
        self.slider_boundwidth.valueChanged.connect(self.slider_boundwidth_changed)
        self.slider_boundheight.valueChanged.connect(self.slider_boundheight_changed)

        self.slider_hue_red_l.valueChanged.connect(self.slider_hue_red_l_changed)
        self.slider_hue_red_h.valueChanged.connect(self.slider_hue_red_h_changed)
        self.slider_saturation_red_l.valueChanged.connect(self.slider_saturation_red_l_changed)
        self.slider_saturation_red_h.valueChanged.connect(self.slider_saturation_red_h_changed)
        self.slider_lightness_red_l.valueChanged.connect(self.slider_lightness_red_l_changed)
        self.slider_lightness_red_h.valueChanged.connect(self.slider_lightness_red_h_changed)

        self.slider_hue_green_l.valueChanged.connect(self.slider_hue_green_l_changed)
        self.slider_hue_green_h.valueChanged.connect(self.slider_hue_green_h_changed)
        self.slider_saturation_green_l.valueChanged.connect(self.slider_saturation_green_l_changed)
        self.slider_saturation_green_h.valueChanged.connect(self.slider_saturation_green_h_changed)
        self.slider_lightness_green_l.valueChanged.connect(self.slider_lightness_green_l_changed)
        self.slider_lightness_green_h.valueChanged.connect(self.slider_lightness_green_h_changed)
        
        self.slider_boundsize.valueChanged.connect(self.slider_boundsize_changed)
        
        self.save_button.clicked.connect(self.save_button_clicked)

    def slider_boundcenter_x_changed(self, value):
        parameter_value = float(value)
        self.label_boundcenter_x.setText(f'boundcenter_x Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('boundcenter_x', parameter_value)

    def slider_boundcenter_y_changed(self, value):
        parameter_value = float(value)
        self.label_boundcenter_y.setText(f'boundcenter_y Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('boundcenter_y', parameter_value)

    def slider_boundwidth_changed(self, value):
        parameter_value = float(value)
        self.label_boundwidth.setText(f'boundwidth Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('boundwidth', parameter_value)

    def slider_boundheight_changed(self, value):
        parameter_value = float(value)
        self.label_boundheight.setText(f'boundheight Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('boundheight', parameter_value)

    def slider_hue_red_l_changed(self, value):
        parameter_value = float(value)
        self.label_hue_red_l.setText(f'hue_red_l Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('hue_red_l', parameter_value)

    def slider_hue_red_h_changed(self, value):
        parameter_value = float(value)
        self.label_hue_red_h.setText(f'hue_red_h Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('hue_red_h', parameter_value)

    def slider_saturation_red_l_changed(self, value):
        parameter_value = float(value)
        self.label_saturation_red_l.setText(f'saturation_red_l Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('saturation_red_l', parameter_value)

    def slider_saturation_red_h_changed(self, value):
        parameter_value = float(value)
        self.label_saturation_red_h.setText(f'saturation_red_h Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('saturation_red_h', parameter_value)

    def slider_lightness_red_l_changed(self, value):
        parameter_value = float(value)
        self.label_lightness_red_l.setText(f'lightness_red_l Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('lightness_red_l', parameter_value)

    def slider_lightness_red_h_changed(self, value):
        parameter_value = float(value)
        self.label_lightness_red_h.setText(f'lightness_red_h Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('lightness_red_h', parameter_value)

    def slider_hue_green_l_changed(self, value):
        parameter_value = float(value)
        self.label_hue_green_l.setText(f'hue_green_l Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('hue_green_l', parameter_value)

    def slider_hue_green_h_changed(self, value):
        parameter_value = float(value)
        self.label_hue_green_h.setText(f'hue_green_h Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('hue_green_h', parameter_value)

    def slider_saturation_green_l_changed(self, value):
        parameter_value = float(value)
        self.label_saturation_green_l.setText(f'saturation_green_l Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('saturation_green_l', parameter_value)

    def slider_saturation_green_h_changed(self, value):
        parameter_value = float(value)
        self.label_saturation_green_h.setText(f'saturation_green_h Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('saturation_green_h', parameter_value)

    def slider_lightness_green_l_changed(self, value):
        parameter_value = float(value)
        self.label_lightness_green_l.setText(f'lightness_green_l Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('lightness_green_l', parameter_value)

    def slider_lightness_green_h_changed(self, value):
        parameter_value = float(value)
        self.label_lightness_green_h.setText(f'lightness_green_h Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('lightness_green_h', parameter_value)

    def save_button_clicked(self):
        self.adjuster_node.save_parameters_to_yaml()
        QMessageBox.information(self, 'Saved', 'Parameters saved to parameters.yaml')

    def slider_boundsize_changed(self, value):
        parameter_value = float(value)
        self.label_boundsize.setText(f'boundsize Value: {parameter_value}')
        self.adjuster_node.adjust_parameters('boundsize', parameter_value)

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
        self.slider_boundcenter_x.setValue(int(parameters.get('boundcenter_x', 480)))
        self.slider_boundcenter_y.setValue(int(parameters.get('boundcenter_y', 320)))
        self.slider_boundwidth.setValue(int(parameters.get('boundwidth', 50)))
        self.slider_boundheight.setValue(int(parameters.get('boundheight', 100)))
   
        self.slider_hue_red_l.setValue(int(parameters.get('hue_red_l', 0)))
        self.slider_hue_red_h.setValue(int(parameters.get('hue_red_h', 0)))
        self.slider_saturation_red_l.setValue(int(parameters.get('saturation_red_l', 0)))
        self.slider_saturation_red_h.setValue(int(parameters.get('saturation_red_h', 50)))
        self.slider_lightness_red_l.setValue(int(parameters.get('lightness_red_l', 230)))
        self.slider_lightness_red_h.setValue(int(parameters.get('lightness_red_h', 255)))
        self.slider_hue_green_l.setValue(int(parameters.get('hue_green_l', 8)))
        self.slider_hue_green_h.setValue(int(parameters.get('hue_green_h', 36)))
        self.slider_saturation_green_l.setValue(int(parameters.get('saturation_green_l', 8)))
        self.slider_saturation_green_h.setValue(int(parameters.get('saturation_green_h', 80)))
        self.slider_lightness_green_l.setValue(int(parameters.get('lightness_green_l', 240)))
        self.slider_lightness_green_h.setValue(int(parameters.get('lightness_green_h', 255)))

        self.slider_boundsize.setValue(int(parameters.get('boundsize', 50)))

        # Update labels
        self.label_boundcenter_x.setText(f'boundcenter_x Value: {parameters.get("boundcenter_x", 480)}')
        self.label_hue_red_h.setText(f'boundcenter_y Value: {parameters.get("boundcenter_y", 320)}')
        self.label_boundwidth.setText(f'boundwidth Value: {parameters.get("boundwidth", 50)}')
        self.label_boundheight.setText(f'boundheight Value: {parameters.get("boundheight", 100)}')

        self.label_hue_red_l.setText(f'hue_red_l Value: {parameters.get("hue_red_l", 0)}')
        self.label_hue_red_h.setText(f'hue_red_h Value: {parameters.get("hue_red_h", 0)}')
        self.label_saturation_red_l.setText(f'saturation_red_l Value: {parameters.get("saturation_red_l", 0)}')
        self.label_saturation_red_h.setText(f'saturation_red_h Value: {parameters.get("saturation_red_h", 50)}')
        self.label_lightness_red_l.setText(f'lightness_red_l Value: {parameters.get("lightness_red_l", 230)}')
        self.label_lightness_red_h.setText(f'lightness_red_h Value: {parameters.get("lightness_red_h", 255)}')
        self.label_hue_green_l.setText(f'hue_green_l Value: {parameters.get("hue_green_l", 8)}')
        self.label_hue_green_h.setText(f'hue_green_h Value: {parameters.get("hue_green_h", 36)}')
        self.label_saturation_green_l.setText(f'saturation_green_l Value: {parameters.get("saturation_green_l", 8)}')
        self.label_saturation_green_h.setText(f'saturation_green_h Value: {parameters.get("saturation_green_h", 80)}')
        self.label_lightness_green_l.setText(f'lightness_green_l Value: {parameters.get("lightness_green_l", 240)}')
        self.label_lightness_green_h.setText(f'lightness_green_h Value: {parameters.get("lightness_green_h", 255)}')

        self.label_boundsize.setText(f'boundsize Value: {parameters.get("boundsize", 50)}')

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
