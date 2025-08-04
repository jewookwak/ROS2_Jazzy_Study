import cv2
import socket
import struct
import numpy as np
import time

class MulticastCameraClient:
    def __init__(self, multicast_group='224.1.1.1', port=10000):
        self.multicast_group = multicast_group
        self.port = port
        self.sock = None
        
    def connect(self):
        """멀티캐스트 그룹에 연결"""
        try:
            # UDP 소켓 생성
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            
            # 소켓 옵션 설정
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            
            # 포트에 바인드 (모든 인터페이스)
            self.sock.bind(('', self.port))
            
            # 멀티캐스트 그룹 가입
            mreq = struct.pack("4sl", socket.inet_aton(self.multicast_group), socket.INADDR_ANY)
            self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
            
            # 수신 버퍼 크기 설정
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024*1024)
            
            print(f"✅ 멀티캐스트 그룹 {self.multicast_group}:{self.port}에 연결됨")
            return True
            
        except Exception as e:
            print(f"❌ 연결 실패: {e}")
            return False
    
    def receive_stream(self):
        """스트림 수신 및 표시"""
        if not self.connect():
            return
            
        print("📺 스트림 수신 시작...")
        print("종료하려면 'q'를 누르거나 Ctrl+C를 입력하세요.")
        
        frame_data = b""
        frame_count = 0
        start_time = time.time()
        
        try:
            while True:
                try:
                    # 데이터 수신 (타임아웃 5초)
                    self.sock.settimeout(5.0)
                    data, addr = self.sock.recvfrom(65536)
                    
                    if data == b"FRAME_END":
                        # 프레임 완료
                        if frame_data:
                            try:
                                # JPEG 디코딩
                                frame = cv2.imdecode(
                                    np.frombuffer(frame_data, np.uint8), 
                                    cv2.IMREAD_COLOR
                                )
                                
                                if frame is not None:
                                    # 색상 채널 순서 수정 (BGR → RGB → BGR)
                                    # 서버에서 RGB→BGR로 변환했지만 JPEG 압축 과정에서 
                                    # 채널 순서가 뒤바뀌는 경우가 있음
                                    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                                    
                                    # 프레임 카운터 및 FPS 표시
                                    frame_count += 1
                                    elapsed_time = time.time() - start_time
                                    
                                    if elapsed_time > 0:
                                        fps = frame_count / elapsed_time
                                        
                                        # 텍스트 오버레이
                                        cv2.putText(frame, f"Frame: {frame_count}", 
                                                  (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                                                  0.7, (0, 255, 0), 2)
                                        cv2.putText(frame, f"FPS: {fps:.1f}", 
                                                  (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 
                                                  0.7, (0, 255, 0), 2)
                                        cv2.putText(frame, f"Server: {addr[0]}", 
                                                  (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 
                                                  0.7, (0, 255, 0), 2)
                                    
                                    # 화면에 표시
                                    cv2.imshow('PiCamera2 Multicast Stream', frame)
                                    
                                    # 매 100프레임마다 상태 출력
                                    if frame_count % 100 == 0:
                                        print(f"📊 수신된 프레임: {frame_count}, FPS: {fps:.1f}")
                                
                                frame_data = b""
                                
                            except Exception as e:
                                print(f"⚠️  프레임 디코딩 오류: {e}")
                                frame_data = b""
                    else:
                        # 프레임 데이터 누적
                        frame_data += data
                    
                    # 키 입력 확인 (1ms 대기)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q') or key == 27:  # 'q' 또는 ESC
                        print("👋 사용자가 종료를 요청했습니다.")
                        break
                        
                except socket.timeout:
                    print("⏱️  서버 응답 없음 (5초 타임아웃)")
                    print("서버가 실행 중인지 확인해주세요.")
                    continue
                    
                except Exception as e:
                    print(f"❌ 수신 오류: {e}")
                    time.sleep(1)
                    continue
                    
        except KeyboardInterrupt:
            print("\n🛑 Ctrl+C로 중단됨")
            
        finally:
            self.cleanup()
    
    def cleanup(self):
        """리소스 정리"""
        try:
            if self.sock:
                # 멀티캐스트 그룹 탈퇴
                mreq = struct.pack("4sl", socket.inet_aton(self.multicast_group), socket.INADDR_ANY)
                self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_DROP_MEMBERSHIP, mreq)
                self.sock.close()
                
            cv2.destroyAllWindows()
            print("✅ 클라이언트가 정상 종료되었습니다.")
            
        except Exception as e:
            print(f"⚠️  정리 중 오류: {e}")

def main():
    """메인 함수"""
    print("🎥 PiCamera2 멀티캐스트 클라이언트")
    print("=" * 40)
    
    # 기본 설정
    multicast_group = '224.1.1.1'
    port = 10000
    
    # 사용자 입력으로 설정 변경 가능
    try:
        user_input = input(f"멀티캐스트 주소 [{multicast_group}]: ").strip()
        if user_input:
            multicast_group = user_input
            
        user_input = input(f"포트 번호 [{port}]: ").strip()
        if user_input:
            port = int(user_input)
            
    except KeyboardInterrupt:
        print("\n종료합니다.")
        return
    except ValueError:
        print("잘못된 포트 번호입니다. 기본값을 사용합니다.")
    
    # 클라이언트 시작
    client = MulticastCameraClient(multicast_group, port)
    client.receive_stream()

if __name__ == "__main__":
    main()