#!/usr/bin/env python3
"""
TEKNOFEST Tabela Tanıma Sistemi
Arial font tabelaları (60cm çap) YOLO ve OCR ile tanır
Parkur aşamalarını belirlemek için kullanılır
"""

import rospy
import cv2
import numpy as np
import threading
import time
import pytesseract
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

# Dart detection sistemini import et
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'dart_detection'))

try:
    from yolo_predictions import YOLO_Pred
    YOLO_AVAILABLE = True
    print("✅ YOLO tabela tanıma modülü yüklendi!")
except ImportError as e:
    print(f"⚠️ YOLO modülü yüklenemedi: {e}")
    YOLO_AVAILABLE = False

class TabelaRecognitionSystem:
    """
    TEKNOFEST Tabela tanıma sistemi
    """
    
    def __init__(self):
        """
        Tabela tanıma sistemi başlatıcı
        """
        # ROS node başlat
        rospy.init_node('tabela_recognition_system', anonymous=True)
        
        # Parametreler
        self.camera_topic = rospy.get_param('~camera_topic', '/camera/image_raw')
        self.detection_confidence = rospy.get_param('~detection_confidence', 0.7)
        self.ocr_enabled = rospy.get_param('~ocr_enabled', True)
        
        # OpenCV bridge
        self.bridge = CvBridge()
        
        # YOLO detector (eğer varsa)
        self.yolo_detector = None
        if YOLO_AVAILABLE:
            try:
                model_path = os.path.join(os.path.dirname(__file__), 
                                        'dart_detection', 'Model', 'weights', 'best.onnx')
                data_path = os.path.join(os.path.dirname(__file__), 
                                       'dart_detection', 'data.yaml')
                
                if os.path.exists(model_path) and os.path.exists(data_path):
                    self.yolo_detector = YOLO_Pred(model_path, data_path)
                    rospy.loginfo("✅ YOLO tabela detector yüklendi")
                else:
                    rospy.logwarn("⚠️ YOLO model dosyaları bulunamadı")
                    
            except Exception as e:
                rospy.logerr(f"❌ YOLO detector yükleme hatası: {e}")
        
        # OCR konfigürasyonu
        if self.ocr_enabled:
            # Tesseract konfigürasyonu
            self.tesseract_config = r'--oem 3 --psm 8 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPQRSTUVWXYZ'
        
        # Bilinen tabela tipleri (TEKNOFEST parkur)
        self.known_signs = {
            'DİK ENGEL': 'obstacle_vertical',
            'TAŞLI YOL': 'rocky_road', 
            'YAN EĞİM': 'side_slope',
            'HIZLANMA': 'acceleration',
            'SIĞI SU': 'shallow_water',
            'TRAFİK KONİLERİ': 'traffic_cones',
            'ENGEBELİ ARAZİ': 'rough_terrain',
            'DİK EĞİM': 'steep_slope',
            'ATIŞ': 'shooting',
            'START': 'start_line',
            'FİNİSH': 'finish_line'
        }
        
        # Detection durumu
        self.current_image = None
        self.last_detection_time = 0
        self.detected_signs = []
        
        # ROS Publishers
        self.sign_detection_pub = rospy.Publisher('/detected_signs', String, queue_size=10)
        self.sign_image_pub = rospy.Publisher('/sign_detection_image', Image, queue_size=1)
        self.marker_pub = rospy.Publisher('/sign_markers', Marker, queue_size=10)
        
        # ROS Subscribers
        rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        rospy.Subscriber('/detect_signs_cmd', String, self.detection_command_callback)
        
        # Threading
        self.processing = False
        self.detection_thread = None
        
        rospy.loginfo("📋 Tabela Tanıma Sistemi başlatıldı")
        rospy.loginfo(f"📷 Kamera topic: {self.camera_topic}")
        rospy.loginfo(f"🔤 OCR aktif: {self.ocr_enabled}")
    
    def image_callback(self, msg):
        """
        Kamera görüntüsü callback
        
        Args:
            msg: sensor_msgs/Image
        """
        try:
            # ROS Image'ı OpenCV formatına çevir
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
            
            # Eğer processing aktif değilse yeni detection başlat
            if not self.processing:
                self.start_detection()
                
        except Exception as e:
            rospy.logerr(f"❌ Image callback hatası: {e}")
    
    def detection_command_callback(self, msg):
        """
        Detection komut callback
        
        Args:
            msg: std_msgs/String
        """
        cmd = msg.data.lower()
        
        if cmd == "start_detection":
            self.start_continuous_detection()
        elif cmd == "stop_detection":
            self.stop_detection()
        elif cmd == "single_detection":
            self.start_detection()
    
    def start_detection(self):
        """
        Tek detection başlat
        """
        if self.current_image is None:
            rospy.logwarn("⚠️ Görüntü mevcut değil")
            return
        
        self.processing = True
        self.detection_thread = threading.Thread(target=self._process_detection, daemon=True)
        self.detection_thread.start()
    
    def start_continuous_detection(self):
        """
        Sürekli detection başlat
        """
        rospy.loginfo("🚀 Sürekli tabela tanıma başlatıldı")
        # Bu mod her image_callback'te otomatik detection yapar
        pass
    
    def stop_detection(self):
        """
        Detection durdur
        """
        self.processing = False
        rospy.loginfo("🛑 Tabela tanıma durduruldu")
    
    def _process_detection(self):
        """
        Ana detection işlemi
        """
        if self.current_image is None:
            return
        
        try:
            image = self.current_image.copy()
            detections = []
            
            # 1. YOLO ile tabela tespiti
            if self.yolo_detector:
                yolo_detections = self._yolo_detection(image)
                detections.extend(yolo_detections)
            
            # 2. Geometric shape detection (dairesel tabelalar için)
            geometric_detections = self._geometric_detection(image)
            detections.extend(geometric_detections)
            
            # 3. OCR ile yazı tanıma
            if self.ocr_enabled and detections:
                self._ocr_recognition(image, detections)
            
            # 4. Sonuçları yayınla
            if detections:
                self._publish_detections(detections, image)
                
        except Exception as e:
            rospy.logerr(f"❌ Detection processing hatası: {e}")
        
        finally:
            self.processing = False
    
    def _yolo_detection(self, image):
        """
        YOLO ile tabela detection
        
        Args:
            image: OpenCV görüntüsü
            
        Returns:
            list: Detection listesi
        """
        detections = []
        
        try:
            # YOLO predictions (dart detection kodundan adapte)
            predicted_image = self.yolo_detector.predictions(image)
            
            # Burada YOLO_Pred sınıfını tabela detection için modifiye etmek gerekiyor
            # Şimdilik basit rectangle detection yapalım
            
        except Exception as e:
            rospy.logerr(f"❌ YOLO detection hatası: {e}")
        
        return detections
    
    def _geometric_detection(self, image):
        """
        Geometric shape detection (60cm çap dairesel tabelalar)
        
        Args:
            image: OpenCV görüntüsü
            
        Returns:
            list: Detection listesi
        """
        detections = []
        
        try:
            # Görüntüyü gri tonlarına çevir
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Gaussian blur uygula
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            
            # Hough Circle Transform
            circles = cv2.HoughCircles(
                blurred,
                cv2.HOUGH_GRADIENT,
                dp=1,
                minDist=100,    # Minimum mesafe between circle centers
                param1=50,      # Edge detection threshold
                param2=30,      # Accumulator threshold
                minRadius=30,   # Minimum circle radius
                maxRadius=200   # Maximum circle radius
            )
            
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                
                for (x, y, r) in circles:
                    # ROI (Region of Interest) çıkar
                    roi = image[max(0, y-r):min(image.shape[0], y+r),
                               max(0, x-r):min(image.shape[1], x+r)]
                    
                    detection = {
                        'type': 'circular_sign',
                        'center': (x, y),
                        'radius': r,
                        'roi': roi,
                        'confidence': 0.8,  # Geometric detection confidence
                        'text': None  # OCR ile doldurulacak
                    }
                    
                    detections.append(detection)
                    
                    rospy.loginfo(f"🔵 Dairesel tabela tespit edildi: ({x}, {y}), R={r}")
            
        except Exception as e:
            rospy.logerr(f"❌ Geometric detection hatası: {e}")
        
        return detections
    
    def _ocr_recognition(self, image, detections):
        """
        OCR ile yazı tanıma
        
        Args:
            image: OpenCV görüntüsü
            detections: Detection listesi (modifiye edilecek)
        """
        try:
            for detection in detections:
                if 'roi' in detection and detection['roi'] is not None:
                    roi = detection['roi']
                    
                    # ROI'yi OCR için hazırla
                    processed_roi = self._preprocess_for_ocr(roi)
                    
                    # Tesseract ile OCR
                    text = pytesseract.image_to_string(
                        processed_roi, 
                        config=self.tesseract_config
                    ).strip().upper()
                    
                    if text:
                        detection['text'] = text
                        
                        # Bilinen tabela tiplerinden eşleştir
                        for known_sign, sign_type in self.known_signs.items():
                            if known_sign in text or text in known_sign:
                                detection['sign_type'] = sign_type
                                detection['confidence'] *= 1.2  # OCR match bonus
                                rospy.loginfo(f"📋 Tabela tanındı: {known_sign} -> {sign_type}")
                                break
                        else:
                            detection['sign_type'] = 'unknown'
                            rospy.loginfo(f"❓ Bilinmeyen tabela: {text}")
                    
        except Exception as e:
            rospy.logerr(f"❌ OCR hatası: {e}")
    
    def _preprocess_for_ocr(self, roi):
        """
        ROI'yi OCR için hazırla
        
        Args:
            roi: Region of interest görüntüsü
            
        Returns:
            np.array: İşlenmiş görüntü
        """
        # Gri tonlarına çevir
        if len(roi.shape) == 3:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi
        
        # Kontrast artır
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(gray)
        
        # Threshold uygula
        _, binary = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        # Noise reduction
        denoised = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, 
                                   cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3)))
        
        # Resize (OCR için optimal boyut)
        height, width = denoised.shape
        if height < 50 or width < 50:
            scale = max(50/height, 50/width)
            new_width = int(width * scale)
            new_height = int(height * scale)
            denoised = cv2.resize(denoised, (new_width, new_height), interpolation=cv2.INTER_CUBIC)
        
        return denoised
    
    def _publish_detections(self, detections, image):
        """
        Detection sonuçlarını yayınla
        
        Args:
            detections: Detection listesi
            image: Orijinal görüntü
        """
        try:
            # Detection listesini JSON formatında yayınla
            detection_data = []
            
            for i, detection in enumerate(detections):
                data = {
                    'id': i,
                    'type': detection.get('type', 'unknown'),
                    'sign_type': detection.get('sign_type', 'unknown'),
                    'text': detection.get('text', ''),
                    'confidence': detection.get('confidence', 0.0),
                    'center': detection.get('center', (0, 0)),
                    'radius': detection.get('radius', 0)
                }
                detection_data.append(data)
            
            # String mesajı olarak yayınla
            detection_msg = String()
            detection_msg.data = str(detection_data)
            self.sign_detection_pub.publish(detection_msg)
            
            # Görselleştirilmiş görüntüyü yayınla
            annotated_image = self._annotate_image(image.copy(), detections)
            image_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            self.sign_image_pub.publish(image_msg)
            
            rospy.loginfo(f"📋 {len(detections)} tabela tespit edildi ve yayınlandı")
            
        except Exception as e:
            rospy.logerr(f"❌ Detection publishing hatası: {e}")
    
    def _annotate_image(self, image, detections):
        """
        Görüntüye detection sonuçlarını çiz
        
        Args:
            image: OpenCV görüntüsü
            detections: Detection listesi
            
        Returns:
            np.array: Annotated görüntü
        """
        for detection in detections:
            center = detection.get('center', (0, 0))
            radius = detection.get('radius', 0)
            text = detection.get('text', 'UNKNOWN')
            confidence = detection.get('confidence', 0.0)
            
            # Daireyi çiz
            cv2.circle(image, center, radius, (0, 255, 0), 3)
            cv2.circle(image, center, 5, (0, 0, 255), -1)
            
            # Metni yazz
            label = f"{text} ({confidence:.2f})"
            label_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
            
            # Text background
            cv2.rectangle(image, 
                         (center[0] - label_size[0]//2, center[1] - radius - 30),
                         (center[0] + label_size[0]//2, center[1] - radius - 5),
                         (0, 255, 0), -1)
            
            # Text
            cv2.putText(image, label,
                       (center[0] - label_size[0]//2, center[1] - radius - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        
        return image
    
    def run(self):
        """
        Ana çalıştırma döngüsü
        """
        rospy.loginfo("🚀 Tabela tanıma sistemi çalışıyor")
        
        try:
            # ROS spin
            rospy.spin()
            
        except KeyboardInterrupt:
            rospy.loginfo("🛑 Tabela tanıma sistemi durduruluyor...")
            
        finally:
            self.stop_detection()

def main():
    """
    Ana fonksiyon
    """
    try:
        tabela_system = TabelaRecognitionSystem()
        tabela_system.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("🛑 ROS interrupted")
    except Exception as e:
        rospy.logerr(f"❌ Tabela tanıma hatası: {e}")

if __name__ == "__main__":
    main()
