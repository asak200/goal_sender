import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import numpy as np
import cv2 as cv
from multiprocessing.pool import ThreadPool
from collections import deque
from dbr import *
import time

# Constants
LICENSE_KEY = "t0068lQAAAJo4Zphb/H5u+hXkMvNYkvJaugGDuxhkxlHomBIS5p+ik4EniQHb+nJT0Etw4jT62Tk0eDNSocYKOJBwBRQjoxY=;t0068lQAAAJk9a1kS3kziyId8qSP5UVnwj3EaWWBtK+Tb+z8pfy0vmn+7H2DaXfKRWaqe5SGIQNwplT7CnjzGG7RCimgStO0="
CAMERA_INDEX = 0
MAX_THREADS = cv.getNumberOfCPUs()


class OptimizedQRCodeScannerNode(Node):
    def __init__(self):
        super().__init__('qr_code_scanner')
        
        # Create a publisher for QR code data
        self.publisher_ = self.create_publisher(String, 'qr_code_data', 10)
        self.msg = String()

        reader = self.initialize_barcode_reader()
        pool = ThreadPool(processes=MAX_THREADS)
        barcode_tasks = deque()

        cap = cv.VideoCapture(CAMERA_INDEX)
        if not cap.isOpened():
            print("Error: Could not open camera.")

        frame_count = 0
        start_time = time.time()

        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("Error: Failed to capture frame.")
                    break

                while barcode_tasks and barcode_tasks[0].ready():
                    results = barcode_tasks.popleft().get()
                    if results:
                        for result in results:
                            self.draw_barcode(frame, result)
                            print(f"Detected barcode: {result.barcode_text}")
                            frame_count += 1

                if len(barcode_tasks) < MAX_THREADS:
                    task = pool.apply_async(self.process_frame, (frame.copy(), reader))
                    barcode_tasks.append(task)

                cv.imshow('Barcode & QR Code Scanner', frame)

                if cv.waitKey(1) & 0xFF == 27:  # ESC key
                    break

                # Calculate and print FPS every 10 seconds
                if time.time() - start_time > 10:
                    fps = frame_count / (time.time() - start_time)
                    print(f"FPS: {fps:.2f}")
                    frame_count = 0
                    start_time = time.time()

        finally:
            cap.release()
            cv.destroyAllWindows()
            pool.close()
            pool.join()

    def initialize_barcode_reader(self):
        BarcodeReader.init_license(LICENSE_KEY)
        return BarcodeReader()

    def process_frame(self, frame, reader):
        try:
            return reader.decode_buffer(frame)
        except BarcodeReaderError as bre:
            print(f"Error decoding barcode: {bre}")
            return None

    def draw_barcode(self, frame, result):
        points = result.localization_result.localization_points
        for i in range(4):
            cv.line(frame, points[i], points[(i+1)%4], (0,255,0), 2)
        cv.putText(frame, result.barcode_text, points[0], cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255))
        
        self.msg.data = result.barcode_text
        self.publisher_.publish(self.msg)

def main(args=None):
    rclpy.init(args=args)
    node = OptimizedQRCodeScannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
