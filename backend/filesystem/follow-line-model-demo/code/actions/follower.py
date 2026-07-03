# pylint: disable=no-member
import os
import py_trees
import sensor_msgs
import cv_bridge
import cv2
import numpy as np
import onnxruntime
import tree_tools

class follower(py_trees.behaviour.Behaviour):

    # Eliminamos 'model_path' de los argumentos para no confundir al creador del árbol
    def __init__(self, name, ports=None):
        """Constructor, se ejecuta al instanciar la clase"""
        super().__init__(name)
        self.logger.debug("%s.__init__()" % (self.__class__.__name__))
        
        # Ahora los puertos se asignan correctamente
        self.ports = ports
        
        # Cargar el modelo ONNX internamente
        try:
            # Buscaremos el modelo en varias ubicaciones posibles del contenedor
            dir_path = os.path.dirname(os.path.realpath(__file__))
            posibles_rutas = [
                os.path.join(dir_path, "follow-line-imitation-master-model-v1.onnx"),
                os.path.join(dir_path, "actions", "follow-line-imitation-master-model-v1.onnx"),
                "/workspace/code/actions/follow-line-imitation-master-model-v1.onnx",
                "/workspace/code/follow-line-imitation-master-model-v1.onnx"
            ]
            
            ruta_modelo = None
            for ruta in posibles_rutas:
                if os.path.exists(ruta):
                    ruta_modelo = ruta
                    self.logger.info(f"Modelo encontrado en: {ruta}")
                    break
                    
            if ruta_modelo is None:
                raise FileNotFoundError(f"No se pudo encontrar el archivo .onnx en ninguna de las rutas: {posibles_rutas}")
            
            self.session = onnxruntime.InferenceSession(
                ruta_modelo, 
                providers=['CUDAExecutionProvider', 'CPUExecutionProvider']
            )
            self.input_name = self.session.get_inputs()[0].name
            self.output_name = self.session.get_outputs()[0].name
            self.logger.info("Modelo ONNX cargado correctamente.")
        except Exception as e:
            self.logger.error(f"Error al cargar el modelo ONNX: {e}")
            self.session = None
            
    def setup(self, **kwargs: int) -> None:
        """Se ejecuta cuando se llama a setup en el árbol"""
        try:
            self.node = kwargs["node"]
        except KeyError as e:
            raise KeyError("No se encontró el nodo de ROS en kwargs") from e

        # Suscripción a la cámara
        self.subscription = self.node.create_subscription(
            sensor_msgs.msg.Image, 
            '/cam_f1_left/image_raw', 
            self.listener_callback, 
            10
        )
        
        self.last_img_ = None
        self.bridge_ = cv_bridge.CvBridge()

    def listener_callback(self, msg):
        """Callback para actualizar la última imagen recibida"""
        self.last_img_ = msg

    def initialise(self) -> None:
        """Se ejecuta al salir de un estado inactivo"""
        self.logger.debug("%s.initialise()" % (self.__class__.__name__))

    def update(self) -> py_trees.common.Status:
        """Lógica principal de inferencia. No debe bloquear."""
        
        if self.session is None or self.last_img_ is None:
            # Si el modelo falló o no hay imagen, enviar velocidad 0 por seguridad
            tree_tools.set_port_content(self.ports["lin_speed"], 0.0)
            tree_tools.set_port_content(self.ports["ang_speed"], 0.0)
            return py_trees.common.Status.FAILURE

        try:
            # 1. Convertir imagen de ROS a OpenCV
            img = self.bridge_.imgmsg_to_cv2(self.last_img_, "bgr8")
            
            # --- NUEVA LÓGICA: DETECTAR SI HAY LÍNEA ROJA ---
            # Convertimos la imagen a espacio de color HSV (mejor para detectar colores)
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            
            # Definimos los rangos del color rojo en HSV (el rojo está en dos extremos del espectro)
            lower_red_1 = np.array([0, 70, 50])
            upper_red_1 = np.array([10, 255, 255])
            lower_red_2 = np.array([170, 70, 50])
            upper_red_2 = np.array([180, 255, 255])
            
            # Creamos máscaras para aislar el rojo
            mask1 = cv2.inRange(hsv, lower_red_1, upper_red_1)
            mask2 = cv2.inRange(hsv, lower_red_2, upper_red_2)
            mask = mask1 + mask2
            
            # Contamos cuántos píxeles rojos hay en la imagen
            pixeles_rojos = cv2.countNonZero(mask)
            
            # Si hay menos de X píxeles rojos, consideramos que la línea se ha perdido
            if pixeles_rojos < 100:  # Puedes ajustar este umbral (100) si es muy sensible
                self.logger.info("Línea roja no detectada. Devolviendo FAILURE.")
                # Importante: devolvemos FAILURE para que el Fallback active el Plan B (girar)
                return py_trees.common.Status.FAILURE
            # ------------------------------------------------

            # 2. Preprocesamiento (Corte, redimensionamiento, normalización)
            if img.shape[0] < 240 or img.shape[1] < 640:
                img = cv2.resize(img, (640, 480))
                
            cropped_img = cv2.resize(img[-240:, :], (640, 240))
            
            img_rgb = cropped_img[:, :, ::-1].astype(np.float32) / 255.0
            img_rgb = np.transpose(img_rgb, (2, 0, 1))
            img_rgb = np.expand_dims(img_rgb, axis=0)
            
            # 3. Inferencia con ONNX
            output = self.session.run([self.output_name], {self.input_name: img_rgb})[0]
            predictions = output.flatten()
            
            # 4. Asignación de velocidades
            if len(predictions) >= 2:
                lin_speed = float(predictions[0])
                ang_speed = float(predictions[1])
            else:
                lin_speed = 4.0
                ang_speed = float(predictions[0])

            # 5. Escribir resultados en los puertos para el nodo SetSpeed
            tree_tools.set_port_content(self.ports["lin_speed"], lin_speed)
            tree_tools.set_port_content(self.ports["ang_speed"], ang_speed)
            
            return py_trees.common.Status.SUCCESS

        except Exception as e:
            self.logger.error(f"Excepción durante la inferencia: {e}")
            tree_tools.set_port_content(self.ports["lin_speed"], 0.0)
            tree_tools.set_port_content(self.ports["ang_speed"], 0.0)
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status: py_trees.common.Status) -> None:
        """Llamado cuando el comportamiento cambia a un estado no en ejecución"""
        self.logger.debug(
            "%s.terminate()[%s->%s]"
            % (self.__class__.__name__, self.status, new_status)
        )