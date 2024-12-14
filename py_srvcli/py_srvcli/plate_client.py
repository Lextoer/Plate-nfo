import rclpy
from rclpy.node import Node
from py_srvcli.srv import PlateInfo

class PlateClient(Node):
    def __init__(self):
        super().__init__('plate_client')
        self.client = self.create_client(PlateInfo, 'process_plates')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.request = PlateInfo.Request()

    def get_valid_input(self, prompt, input_type):
        """Kullanıcıdan geçerli bir girdi alın."""
        while True:
            try:
                user_input = input(prompt).strip()  # Kullanıcı girişini al ve boşlukları temizle
                if not user_input:  # Eğer giriş boşsa uyarı ver
                    print("Bu alan boş bırakılamaz. Lütfen geçerli bir değer girin.")
                    continue
                return input_type(user_input)  # Girişi istenen tipe dönüştür ve geri döndür
            except ValueError:
                print(f"Lütfen geçerli bir {input_type.__name__} değeri girin.")

    def send_request(self):
        while True:
            try:
                num_plates = self.get_valid_input('Enter the number of plates (Ctrl+Z to quit): ', int)

                self.request.names = []
                self.request.confidences = []
                self.request.x_positions = []
                self.request.y_positions = []

                for i in range(num_plates):
                    name = self.get_valid_input(f'Enter name of plate {i+1}: ', str)
                    confidence = self.get_valid_input(f'Enter confidence of plate {i+1}: ', float)
                    x = self.get_valid_input(f'Enter x position of plate {i+1}: ', float)
                    y = self.get_valid_input(f'Enter y position of plate {i+1}: ', float)

                    self.request.names.append(name)
                    self.request.confidences.append(confidence)
                    self.request.x_positions.append(x)
                    self.request.y_positions.append(y)

                # Servis çağrısı yap
                self.future = self.client.call_async(self.request)
                rclpy.spin_until_future_complete(self, self.future)

                # İşlemin gerçekleştirildiğini bildir
                if self.future.result() is not None:
                    self.get_logger().info('The closest plate has been successfully processed and published.')
                else:
                    self.get_logger().error('Failed to process the plates.')

            except KeyboardInterrupt:
                print("\nProgram terminated by user.")
                break

def main():
    rclpy.init()
    node = PlateClient()
    node.send_request()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
