import rclpy
from rclpy.node import Node
import cv2
# from pyzbar.pyzbar import decode
from std_msgs.msg import String

class QRCodeScannerNode(Node):
    def __init__(self):
        super().__init__('qr_code_scanner')
        
        # Create a publisher for QR code data
        self.qr_order_pub = self.create_publisher(String, 'qr_order', 10)

        # Initialize camera
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("Could not open camera.")
            rclpy.shutdown()

        self.prev_qr = None
        
        # Create a timer to capture and process frames at 10Hz
        self.timer = self.create_timer(0.02, self.timer_callback)

        self.get_logger().info("qr_code_scanner initilized")
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Could not read frame.")
            return
        
        # Decode QR codes from the frame
        # qr_codes = decode(frame)
        
        # for qr_code in qr_codes:
        #     qr_data = qr_code.data.decode('utf-8')
        #     if self.prev_qr == qr_data:
        #         continue
        #     self.prev_qr = qr_data
        #     # self.get_logger().info(f"QR Code detected: {self.prev_qr}")
        #     self.get_logger().info(f"QR Code noooooow: {qr_data}")
            
        #     # Publish the QR code data
        #     msg = String()
        #     msg.data = qr_data
        #     self.qr_order_pub.publish(msg)
        
        # # Display the frame with QR codes (optional, for debugging)
        # frame_with_qr = self.draw_qr_codes(frame, qr_codes)
        cv2.imshow('QR Code Scanner', frame)
        cv2.waitKey(1)  # Needed to keep the window open
        
        
        # if len(qr_codes) == 0:
        #     return
        # qr_data = qr_codes[0].data.decode('utf-8')
        # if self.prev_qr == qr_data:
        #     return
        # self.prev_qr = qr_data
        # # self.get_logger().info(f"QR Code detected: {self.prev_qr}")
        # self.get_logger().info(f"QR Code noooooow: {qr_data}")
        
        # # Publish the QR code data
        # msg = String()
        # msg.data = qr_data
        # self.qr_order_pub.publish(msg)
    
    def draw_qr_codes(self, frame, qr_codes):
        for qr_code in qr_codes:
            
            x, y, w, h = qr_code.rect
            qr_data = qr_code.data.decode('utf-8')
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(frame, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.9, (255, 0, 0), 2)
        return frame
    
    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = QRCodeScannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()