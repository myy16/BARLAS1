#!/usr/bin/env python3
"""
TEKNOFEST Tabela Tanıma Test Sistemi
ROS olmadan kamera ile tabela tanıma testi
"""

import cv2
import numpy as np
import time
import threading
import os
import sys
import json
from queue import Queue

# Tesseract OCR (opsiyonel)
try:
    import pytesseract
    OCR_AVAILABLE = True
    print("✅ OCR (Tesseract) yüklendi")
except ImportError:
    OCR_AVAILABLE = False
    print("⚠️ OCR (Tesseract) yüklenemedi")

# YOLO detector import
sys.path.append(os.path.join(os.path.dirname(__file__), 'dart_detection'))

try:
    from yolo_predictions import YOLO_Pred
    YOLO_AVAILABLE = True
    print("✅ YOLO tabela detector yüklendi")
except ImportError as e:
    YOLO_AVAILABLE = False
    print(f"⚠️ YOLO detector yüklenemedi: {e}")

class TabelaTestSystem:
    """
    Tabela tanıma test sistemi (ROS-free)
    """
    
    def __init__(self, camera_id=0):
        """
        Test sistem başlatıcı
        
        Args:
            camera_id: Kamera ID (0=USB kamera)
        """
        self.camera_id = camera_id
        self.cap = None
        
        # YOLO detector
        self.yolo_detector = None
        if YOLO_AVAILABLE:
            try:
                model_path = os.path.join(os.path.dirname(__file__), 
                                        'dart_detection', 'Model', 'weights', 'best.onnx')
                data_path = os.path.join(os.path.dirname(__file__), 
                                       'dart_detection', 'data.yaml')
                
                if os.path.exists(model_path) and os.path.exists(data_path):
                    self.yolo_detector = YOLO_Pred(model_path, data_path)
                    print("✅ YOLO model yüklendi")
                else:
                    print("⚠️ YOLO model dosyaları bulunamadı")
            except Exception as e:
                print(f"❌ YOLO yükleme hatası: {e}")
        
        # Bilinen tabela tipleri
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
            'FİNİSH': 'finish_line',
            'HIZLANDIR': 'acceleration',
            'YAVASLA': 'decelerate',
            'DUR': 'stop',
            'DEVAM': 'continue'
        }
        
        # OCR config
        if OCR_AVAILABLE:
            self.tesseract_config = r'--oem 3 --psm 8 -c tessedit_char_whitelist=ABCDEFGHIJKLMNOPRSTUVWXYZİĞÜŞÇÖ'
        
        # Sonuçlar
        self.detection_results = Queue()
        self.running = False
        
        print("📋 Tabela Test Sistemi hazır")
    
    def initialize_camera(self):
        """
        Kamera başlat
        """
        try:
            self.cap = cv2.VideoCapture(self.camera_id)
            
            if not self.cap.isOpened():
                print(f"❌ Kamera {self.camera_id} açılamadı")
                return False
            
            # Kamera ayarları
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
            
            print(f"📷 Kamera {self.camera_id} başlatıldı")
            return True
            
        except Exception as e:
            print(f"❌ Kamera başlatma hatası: {e}")
            return False
    
    def detect_circular_signs(self, image):
        """
        Dairesel tabela detection (HoughCircles)
        
        Args:
            image: OpenCV görüntüsü
            
        Returns:
            list: Detection listesi
        """
        detections = []
        
        try:
            # Gri tonlarına çevir
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Gaussian blur
            blurred = cv2.GaussianBlur(gray, (9, 9), 2)
            
            # Hough Circle Transform
            circles = cv2.HoughCircles(
                blurred,
                cv2.HOUGH_GRADIENT,
                dp=1,
                minDist=80,     # Daireler arası minimum mesafe
                param1=50,      # Edge detection threshold
                param2=35,      # Accumulator threshold  
                minRadius=25,   # Minimum yarıçap
                maxRadius=150   # Maximum yarıçap
            )
            
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                
                for (x, y, r) in circles:
                    # Görüntü sınırlarını kontrol et
                    if (x - r >= 0 and y - r >= 0 and 
                        x + r < image.shape[1] and y + r < image.shape[0]):
                        
                        # ROI çıkar
                        roi = image[y-r:y+r, x-r:x+r]
                        
                        detection = {
                            'type': 'circular_sign',
                            'center': (x, y),
                            'radius': r,
                            'roi': roi,
                            'confidence': 0.8,
                            'text': None,
                            'sign_type': 'unknown'
                        }
                        
                        detections.append(detection)
                        print(f"🔵 Dairesel tabela tespit edildi: ({x}, {y}), R={r}")
            
        except Exception as e:
            print(f"❌ Circular detection hatası: {e}")
        
        return detections
    
    def recognize_text_ocr(self, roi):
        """
        OCR ile metin tanıma
        
        Args:
            roi: Region of interest
            
        Returns:
            str: Tanınan metin
        """
        if not OCR_AVAILABLE:
            return None
        
        try:
            # ROI'yi OCR için hazırla
            processed = self.preprocess_for_ocr(roi)
            
            # Tesseract OCR
            text = pytesseract.image_to_string(processed, config=self.tesseract_config)
            text = text.strip().upper()
            
            # Türkçe karakter düzeltmeleri
            text = text.replace('I', 'İ')  # Tesseract I/İ karışıklığı
            
            return text if len(text) > 0 else None
            
        except Exception as e:
            print(f"❌ OCR hatası: {e}")
            return None
    
    def preprocess_for_ocr(self, roi):
        """
        OCR için görüntü hazırlama
        
        Args:
            roi: Region of interest
            
        Returns:
            np.array: İşlenmiş görüntü
        """
        # Gri tonlarına çevir
        if len(roi.shape) == 3:
            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        else:
            gray = roi
        
        # Kontrast artırma
        clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))
        enhanced = clahe.apply(gray)
        
        # Adaptive threshold
        binary = cv2.adaptiveThreshold(enhanced, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                                      cv2.THRESH_BINARY, 11, 2)
        
        # Morfolojik işlemler
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel)
        
        # Boyut optimizasyonu
        height, width = cleaned.shape
        if height < 60 or width < 60:
            scale = max(60/height, 60/width)
            new_width = int(width * scale)
            new_height = int(height * scale)
            cleaned = cv2.resize(cleaned, (new_width, new_height), 
                               interpolation=cv2.INTER_CUBIC)
        
        return cleaned
    
    def match_known_signs(self, text):
        """
        Bilinen tabela tiplerine eşleştir
        
        Args:
            text: OCR metni
            
        Returns:
            tuple: (sign_type, confidence)
        """
        if not text:
            return 'unknown', 0.0
        
        text = text.upper().strip()
        
        # Tam eşleşme
        for known_sign, sign_type in self.known_signs.items():
            if text == known_sign:
                return sign_type, 1.0
        
        # Kısmi eşleşme  
        best_match = 'unknown'
        best_confidence = 0.0
        
        for known_sign, sign_type in self.known_signs.items():
            # İçerik kontrolü
            if known_sign in text or text in known_sign:
                confidence = min(len(text), len(known_sign)) / max(len(text), len(known_sign))
                if confidence > best_confidence:
                    best_confidence = confidence
                    best_match = sign_type
        
        return best_match, best_confidence
    
    def annotate_image(self, image, detections):
        """
        Görüntüye detection sonuçlarını çiz
        
        Args:
            image: OpenCV görüntüsü
            detections: Detection listesi
            
        Returns:
            np.array: İşaretli görüntü
        """
        annotated = image.copy()
        
        for detection in detections:
            center = detection['center']
            radius = detection['radius']
            text = detection.get('text', 'UNKNOWN')
            confidence = detection['confidence']
            sign_type = detection.get('sign_type', 'unknown')
            
            # Daireyi çiz
            color = (0, 255, 0) if sign_type != 'unknown' else (0, 255, 255)
            cv2.circle(annotated, center, radius, color, 3)
            cv2.circle(annotated, center, 5, (0, 0, 255), -1)
            
            # Label hazırla
            if text and text != 'UNKNOWN':
                label = f"{text} ({sign_type}) {confidence:.2f}"
            else:
                label = f"CIRCLE ({sign_type}) {confidence:.2f}"
            
            # Text background ve yazısı
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 1
            
            (text_width, text_height), baseline = cv2.getTextSize(label, font, font_scale, thickness)
            
            # Background rectangle
            cv2.rectangle(annotated,
                         (center[0] - text_width//2 - 5, center[1] - radius - text_height - 15),
                         (center[0] + text_width//2 + 5, center[1] - radius - 5),
                         color, -1)
            
            # Text
            cv2.putText(annotated, label,
                       (center[0] - text_width//2, center[1] - radius - 10),
                       font, font_scale, (0, 0, 0), thickness)
        
        return annotated
    
    def save_detection_result(self, detections, image):
        """
        Detection sonucunu kaydet
        
        Args:
            detections: Detection listesi
            image: Orijinal görüntü
        """
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        
        # Sonuç bilgilerini hazırla
        result_data = {
            'timestamp': timestamp,
            'detections': []
        }
        
        for i, detection in enumerate(detections):
            det_data = {
                'id': i,
                'type': detection['type'],
                'sign_type': detection['sign_type'],
                'text': detection.get('text', ''),
                'confidence': detection['confidence'],
                'center': detection['center'],
                'radius': detection['radius']
            }
            result_data['detections'].append(det_data)
        
        # JSON olarak kaydet
        results_dir = os.path.join(os.path.dirname(__file__), 'results')
        os.makedirs(results_dir, exist_ok=True)
        
        json_path = os.path.join(results_dir, f'tabela_detection_{timestamp}.json')
        with open(json_path, 'w', encoding='utf-8') as f:
            json.dump(result_data, f, ensure_ascii=False, indent=2)
        
        # Görüntüyü kaydet
        annotated_image = self.annotate_image(image, detections)
        image_path = os.path.join(results_dir, f'tabela_detection_{timestamp}.jpg')
        cv2.imwrite(image_path, annotated_image)
        
        print(f"💾 Sonuçlar kaydedildi: {json_path}")
        print(f"🖼️ Görüntü kaydedildi: {image_path}")
    
    def run_test(self, duration=60):
        """
        Test çalıştır
        
        Args:
            duration: Test süresi (saniye)
        """
        if not self.initialize_camera():
            return False
        
        print(f"🚀 Tabela tanıma testi başlatıldı ({duration} saniye)")
        print("📋 Kontroller:")
        print("  SPACE: Anlık detection")
        print("  S: Sonucu kaydet")
        print("  Q: Çıkış")
        
        self.running = True
        start_time = time.time()
        detection_count = 0
        
        try:
            while self.running and (time.time() - start_time) < duration:
                ret, frame = self.cap.read()
                if not ret:
                    print("❌ Kamera görüntüsü alınamadı")
                    break
                
                # Her frame'de otomatik detection (hafif)
                detections = self.detect_circular_signs(frame)
                
                # OCR sadece detection varsa
                if detections and OCR_AVAILABLE:
                    for detection in detections:
                        text = self.recognize_text_ocr(detection['roi'])
                        if text:
                            detection['text'] = text
                            sign_type, confidence_bonus = self.match_known_signs(text)
                            detection['sign_type'] = sign_type
                            detection['confidence'] *= (1.0 + confidence_bonus * 0.5)
                
                # Görüntüyü göster
                display_image = self.annotate_image(frame, detections)
                
                # Info overlay
                info_text = [
                    f"Test süresi: {int(time.time() - start_time)}/{duration}s",
                    f"Detection sayısı: {len(detections)}",
                    f"Toplam detection: {detection_count}"
                ]
                
                for i, text in enumerate(info_text):
                    cv2.putText(display_image, text, (10, 30 + i*20),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                
                cv2.imshow('Tabela Tanıma Testi', display_image)
                
                # Klavye kontrolleri
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    print("🛑 Çıkış")
                    break
                elif key == ord(' '):
                    print("📸 Anlık detection")
                    if detections:
                        print(f"✅ {len(detections)} tabela tespit edildi")
                        for i, det in enumerate(detections):
                            print(f"  {i+1}. {det.get('text', 'NO_TEXT')} - {det['sign_type']} ({det['confidence']:.2f})")
                elif key == ord('s'):
                    if detections:
                        self.save_detection_result(detections, frame)
                        detection_count += len(detections)
                
                # FPS limiti
                time.sleep(0.03)  # ~30 FPS
        
        except KeyboardInterrupt:
            print("🛑 Klavye ile durduruldu")
        
        finally:
            self.cleanup()
            print(f"📊 Test tamamlandı - Toplam {detection_count} detection")
        
        return True
    
    def cleanup(self):
        """
        Temizlik
        """
        self.running = False
        
        if self.cap:
            self.cap.release()
        
        cv2.destroyAllWindows()
        print("🧹 Temizlik tamamlandı")

def main():
    """
    Ana test fonksiyonu
    """
    print("🎯 TEKNOFEST Tabela Tanıma Test Sistemi")
    print("=" * 50)
    
    # Test sistemi oluştur
    test_system = TabelaTestSystem(camera_id=0)
    
    # Test çalıştır
    try:
        success = test_system.run_test(duration=120)  # 2 dakika test
        
        if success:
            print("✅ Test başarıyla tamamlandı")
        else:
            print("❌ Test başarısız")
            
    except Exception as e:
        print(f"❌ Test hatası: {e}")
    
    finally:
        test_system.cleanup()

if __name__ == "__main__":
    main()
