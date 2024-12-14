import rclpy
from rclpy.node import Node
from py_srvcli.msg import PlateData
from datetime import datetime
from tabulate import tabulate
import json
import os
import subprocess

class PlateSubscriber(Node):
    def __init__(self):
        super().__init__('plate_subscriber')
        self.subscription = self.create_subscription(
            PlateData,
            'data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # JSON dosyasının yolu
        self.json_file_path = os.path.expanduser("~/ros2_ws/plate_data.json")

        # JSON dosyasını kontrol et veya başlat
        try:
            os.makedirs(os.path.dirname(self.json_file_path), exist_ok=True)
            if not os.path.exists(self.json_file_path):  # Dosya yoksa oluştur
                with open(self.json_file_path, 'w') as json_file:
                    json.dump([], json_file, ensure_ascii=False, indent=4)
                self.get_logger().info("JSON file created successfully.")
            else:  # Dosya varsa bozuk olup olmadığını kontrol et
                with open(self.json_file_path, 'r') as json_file:
                    try:
                        json.load(json_file)  # JSON'u yüklemeyi dene
                    except json.JSONDecodeError:
                        self.get_logger().warning("JSON file is corrupted. Resetting file.")
                        with open(self.json_file_path, 'w') as reset_file:
                            json.dump([], reset_file, ensure_ascii=False, indent=4)
        except IOError as e:
            self.get_logger().error(f"Failed to create or access JSON file: {e}")

        # Subscriber başlatıldığında bilgi mesajı
        self.get_logger().info('PlateSubscriber is active and listening to the "data" topic.')

    def listener_callback(self, msg):
        try:
            # Yayınlanma tarihi ve saatini al
            current_date = datetime.now().strftime('%Y-%m-%d')
            current_time = datetime.now().strftime('%H:%M:%S')

            # Veriyi tabloya eklemek için hazırlanmış bir liste
            data = [
                ["Date", current_date],
                ["Time", current_time],
                ["Plate Name", msg.name],
                ["Confidence", msg.confidence],
                ["Distance", msg.distance]
            ]

            # Tabloyu oluştur
            table = tabulate(data, headers=["Field", "Value"], tablefmt="grid")
            print("\n" + table)

            # Confidence değerine göre durumu belirle
            if msg.confidence > 90:
                status = "Tespit edildi!"
            elif 80 < msg.confidence <= 90:
                status = "Tespit edildi, net değil!"
            else:
                status = "İşleme alınamadı!"

            # Durumu tablo altına yazdır
            print(f"\nDurum: {status}\n")

            # JSON formatında veri oluştur
            json_data = {
                "date": current_date,
                "time": current_time,
                "plate_name": msg.name,
                "confidence": msg.confidence,
                "distance": msg.distance,
                "status": status
            }

            # JSON dosyasına düzenli bir şekilde liste halinde yazma
            try:
                with open(self.json_file_path, 'r+', encoding='utf-8') as json_file:
                    file_data = json.load(json_file)  # Mevcut veriyi oku
                    file_data.append(json_data)  # Yeni veriyi ekle
                    json_file.seek(0)  # Dosyanın başına dön
                    json.dump(file_data, json_file, ensure_ascii=False, indent=4)  # Türkçe karakter desteği
                self.get_logger().info("Data saved to JSON file successfully.")

                # JSON dosyasını küçük bir pencere olarak aç
                subprocess.Popen(["xdg-open", self.json_file_path])
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Failed to decode JSON file: {e}")
            except IOError as e:
                self.get_logger().error(f"Failed to write to JSON file: {e}")

        except Exception as e:
            self.get_logger().error(f"An error occurred in listener_callback: {e}")


def main():
    rclpy.init()
    node = PlateSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Subscriber manually interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
