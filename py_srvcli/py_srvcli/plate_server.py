import rclpy
from rclpy.node import Node
from py_srvcli.srv import PlateInfo
from py_srvcli.msg import PlateData
from math import sqrt

class PlateServer(Node):
    def __init__(self):
        super().__init__('plate_server')
        self.publisher = self.create_publisher(PlateData, 'data', 10)

        # Servisi oluştur
        self.srv = self.create_service(PlateInfo, 'process_plates', self.process_callback)

        # Servisin hazır olduğuna dair bilgi yazdır
        self.get_logger().info('PlateInfo service is ready and waiting for requests...')

    def process_callback(self, request, response):
        min_distance = float('inf')
        closest_plate = None
        closest_confidence = None

        for i in range(len(request.names)):
            distance = sqrt(request.x_positions[i]**2 + request.y_positions[i]**2)
            if distance < min_distance:
                min_distance = distance
                closest_plate = request.names[i]
                closest_confidence = request.confidences[i]

        response.closest_plate = closest_plate
        response.confidence = closest_confidence
        response.distance = min_distance

        # Publish data to 'data' topic
        msg = PlateData()
        msg.name = closest_plate
        msg.confidence = closest_confidence
        msg.distance = min_distance
        self.publisher.publish(msg)

        self.get_logger().info(f"Published Plate: {msg.name}, Confidence: {msg.confidence}, Distance: {msg.distance}")
        return response

def main():
    rclpy.init()
    node = PlateServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
