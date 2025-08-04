from picamera2 import Picamera2
import cv2
from PIL import Image
import time
import socket
import struct
import numpy as np

# LCD 모듈 import 시도
try:
    from pinky_lcd import LCD
    LCD_AVAILABLE = True
    print("✅ LCD 모듈을 찾았습니다.")
except ImportError:
    LCD_AVAILABLE = False
    print("⚠️  LCD 모듈을 찾을 수 없습니다. LCD 없이 실행합니다.")

class PiCamera2MulticastServer:
    def __init__(self, multicast_group='224.1.1.1', port=10000):
        self.multicast_group = multicast_group
        self.port = port
        
        # UDP 멀티캐스트 소켓 설정
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        
        # LCD 초기화 (사용 가능할 경우에만)
        if LCD_AVAILABLE:
            self.lcd = LCD()
            print("✅ LCD 초기화 완료")
        else:
            self.lcd = None
            print("ℹ️  LCD 없이 실행")
        
        # PiCamera2 초기화
        self.picam2 = Picamera2()
        self.picam2.configure(self.picam2.create_preview_configuration(
            main={"format": 'RGB888', "size": (640, 480)}
        ))
        
    def start_streaming(self):
        print(f"PiCamera2 멀티캐스트 서버 시작: {self.multicast_group}:{self.port}")
        
        self.picam2.start()
        
        try:
            while True:
                # 원본 코드와 정확히 동일한 처리
                frame = self.picam2.capture_array()
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # 원본과 동일
                rotate_frame = cv2.rotate(rgb_frame, cv2.ROTATE_180)
                flipped_frame = cv2.flip(rotate_frame, 1)
                
                # LCD에 표시 (원본과 동일)
                if self.lcd:
                    show_frame = Image.fromarray(flipped_frame)  # RGB 그대로
                    self.lcd.img_show(show_frame)
                
                # UDP 전송을 위해 BGR로 변환 후 JPEG 인코딩
                bgr_for_transmission = cv2.cvtColor(flipped_frame, cv2.COLOR_RGB2BGR)
                _, buffer = cv2.imencode('.jpg', bgr_for_transmission, 
                                       [cv2.IMWRITE_JPEG_QUALITY, 95])  # 고품질 설정
                data = buffer.tobytes()
                
                # 패킷 크기 제한하여 전송
                packet_size = 60000
                for i in range(0, len(data), packet_size):
                    packet = data[i:i + packet_size]
                    self.sock.sendto(packet, (self.multicast_group, self.port))
                
                # 프레임 끝 신호
                self.sock.sendto(b"FRAME_END", (self.multicast_group, self.port))
                
                # 원본 코드와 동일한 대기
                cv2.waitKey(1)
                
        except KeyboardInterrupt:
            print("\n서버 중단 중...")
        finally:
            self.cleanup()
    
    def cleanup(self):
        """리소스 정리"""
        self.picam2.close()
        self.sock.close()
        print("서버가 종료되었습니다.")

# 멀티캐스트 클라이언트
class MulticastCameraClient:
    def __init__(self, multicast_group='224.1.1.1', port=10000):
        self.multicast_group = multicast_group
        self.port = port
        
    def receive_stream(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('', self.port))
        
        mreq = struct.pack("4sl", socket.inet_aton(self.multicast_group), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        
        print(f"멀티캐스트 클라이언트 시작: {self.multicast_group}:{self.port}")
        print("'q'를 누르면 종료됩니다.")
        
        frame_data = b""
        
        try:
            while True:
                data, addr = sock.recvfrom(65536)
                
                if data == b"FRAME_END":
                    if frame_data:
                        try:
                            frame = cv2.imdecode(np.frombuffer(frame_data, np.uint8), cv2.IMREAD_COLOR)
                            if frame is not None:
                                cv2.imshow('PiCamera2 Stream', frame)
                            frame_data = b""
                            
                            if cv2.waitKey(1) & 0xFF == ord('q'):
                                break
                        except Exception as e:
                            print(f"프레임 디코딩 오류: {e}")
                            frame_data = b""
                else:
                    frame_data += data
                    
        except KeyboardInterrupt:
            print("\n클라이언트 중단 중...")
        finally:
            sock.close()
            cv2.destroyAllWindows()
            print("클라이언트가 종료되었습니다.")

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1 and sys.argv[1] == 'client':
        # 클라이언트 모드로 실행
        client = MulticastCameraClient()
        client.receive_stream()
    else:
        # 기본값: 서버 모드로 실행 (원본 코드 기반)
        server = PiCamera2MulticastServer()
        server.start_streaming()