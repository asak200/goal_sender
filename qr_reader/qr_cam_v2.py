import rclpy
from rclpy.node import Node
import cv2
from pyzbar.pyzbar import decode
from std_msgs.msg import String
import queue
import threading

class OptimizedQRCodeScannerNode(Node):
    def __init__(self):
        super().__init__('optimized_qr_code_scanner')
        
        # Create a publisher for QR code data
        self.publisher_ = self.create_publisher(String, 'qr_code_data', 10)
        
        # Initialize camera
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")
            rclpy.shutdown()
        
        # Reduce frame rate to lighten processing load
        # self.cap.set(cv2.CAP_PROP_FPS, 15)
        
        # Initialize variables
        self.prev_qr = None
        self.frame_queue = queue.Queue()
        self.frame_count = 0
        
        # Create a timer to process frames at a specified rate
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            self.process_frame(frame)
    
    def process_frame(self, frame):
        # Convert frame to grayscale to reduce computational load
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Decode QR codes from the grayscale frame
        qr_codes = decode(gray_frame)
        
        if len(qr_codes) == 0:
            return

        qr_data = qr_codes[0].data.decode('utf-8')
        
        # Check if the detected QR code is different from the previous one
        if self.prev_qr == qr_data:
            return
        
        self.prev_qr = qr_data
        self.get_logger().info(f"QR Code detected: {qr_data}")
        
        # Publish the QR code data
        msg = String()
        msg.data = qr_data
        self.publisher_.publish(msg)
    
    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = OptimizedQRCodeScannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()