#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from community_perception.srv import LicensePlateOCR, LicensePlateOCRResponse
import os
import re
import time
import threading
from datetime import datetime
from paddleocr import PaddleOCR

class LicensePlateOCRServer:
    def __init__(self):
        rospy.init_node('license_plate_ocr_server')
        self.bridge = CvBridge()
        self.conf = rospy.get_param('~conf', 0.8)
        # ==================== 图片保存路径 ====================
        self.save_dir = rospy.get_param('~save_dir', os.path.expanduser('~/.ros/community/license_plate_ocr'))
        os.makedirs(self.save_dir, exist_ok=True)
        rospy.loginfo(f"识别结果图片将保存到: {self.save_dir}")
        
        # ==================== 简化配置 ====================
        self.use_center_crop_only = rospy.get_param('~use_center_crop_only', True)
        self.crop_ratio = rospy.get_param('~crop_ratio', 0.6)  # 修改默认值为0.8 (80%)
        
        # ==================== 并发控制 ====================
        self.processing_lock = threading.Lock()
        self.ocr_health = True  # OCR引擎健康状态
        
        # ==================== 初始化PaddleOCR ====================
        self.init_paddle_ocr()
        
        # 创建服务
        self.service = rospy.Service('license_plate_ocr', LicensePlateOCR, self.handle_ocr_request)
        rospy.loginfo("License Plate OCR Server 已启动，等待请求...")
        rospy.loginfo("使用简化模式：直接处理图像中心区域 (x轴60%，y轴全高)")
    
    def init_paddle_ocr(self, max_retries=3):
        """初始化PaddleOCR - 带健康检查和重试"""
        for attempt in range(max_retries):
            try:
                self.ocr = PaddleOCR(
                    use_angle_cls=False,
                    lang='ch',
                    use_gpu=False,
                    show_log=False,
                    drop_score=self.conf,
                    rec_batch_num=1
                )
                rospy.loginfo(f"PaddleOCR初始化成功 (尝试 {attempt+1}/{max_retries})")
                self.ocr_health = True
                return True
            except Exception as e:
                rospy.logerr(f"PaddleOCR初始化失败 (尝试 {attempt+1}/{max_retries}): {e}")
                if attempt < max_retries - 1:
                    rospy.loginfo("等待2秒后重试...")
                    time.sleep(2)
                else:
                    rospy.logerr("PaddleOCR初始化彻底失败")
                    self.ocr_health = False
                    return False
    
    def check_ocr_health(self):
        """检查OCR引擎健康状态，必要时重新初始化"""
        if not self.ocr_health:
            rospy.logwarn("OCR引擎不健康，尝试重新初始化...")
            return self.init_paddle_ocr(max_retries=2)
        return True
    
    # 修改点1：重写裁剪逻辑（y轴不裁剪，x轴70%）
    def get_center_crop(self, image):
        """获取图像中心区域 - 仅x轴裁剪（y轴保持完整）"""
        height, width = image.shape[:2]
        
        # x轴裁剪：中心70%区域（宽度 = width * 0.7）
        crop_width = int(width * self.crop_ratio)
        # y轴不裁剪：高度 = 原图高度
        crop_height = height
        
        # 计算x轴中心位置
        x1 = max(0, width // 2 - crop_width // 2)
        x2 = min(width, x1 + crop_width)
        # y轴：从0到height（不裁剪）
        y1, y2 = 0, height
        
        rospy.loginfo(f"裁剪区域: {width}x{height} -> [{x1},{y1},{x2},{y2}] (x轴60%)")
        return image[y1:y2, x1:x2], [x1, y1, x2, y2]
    
    def enhance_image(self, image):
        """图像增强预处理"""
        try:
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            else:
                gray = image.copy()
            
            clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
            enhanced = clahe.apply(gray)
            
            kernel = np.array([[-1, -1, -1],
                               [-1,  9, -1],
                               [-1, -1, -1]])
            sharpened = cv2.filter2D(enhanced, -1, kernel)
            
            return cv2.cvtColor(sharpened, cv2.COLOR_GRAY2BGR)
        except Exception as e:
            rospy.logwarn(f"图像增强失败: {e}")
            return image
    
    def safe_ocr_process(self, image_path):
        """安全的OCR处理，带错误恢复"""
        if not self.check_ocr_health():
            return None
        
        max_attempts = 2
        for attempt in range(max_attempts):
            try:
                result = self.ocr.ocr(image_path, cls=False)
                
                if result is None:
                    rospy.logwarn(f"OCR返回None (尝试 {attempt+1}/{max_attempts})")
                    continue
                
                if not isinstance(result, list) or len(result) == 0:
                    rospy.logdebug(f"OCR返回空列表 (尝试 {attempt+1}/{max_attempts})")
                    continue
                
                texts = []
                scores = []
                
                for line in result:
                    if line is None:
                        continue
                    for word_info in line:
                        if word_info is None or len(word_info) < 2:
                            continue
                        text_info = word_info[1]
                        if text_info is None or len(text_info) < 2:
                            continue
                        
                        text = text_info[0]
                        score = text_info[1]
                        
                        if text and isinstance(score, (int, float)):
                            texts.append(text)
                            scores.append(score)
                
                if texts:
                    return {
                        'texts': texts,
                        'scores': scores,
                        'success': True
                    }
                    
            except Exception as e:
                error_msg = str(e)
                rospy.logwarn(f"OCR处理失败 (尝试 {attempt+1}/{max_attempts}): {error_msg}")
                
                if "could not execute a primitive" in error_msg:
                    rospy.logwarn("检测到PaddleOCR内部错误，将重新初始化引擎...")
                    self.ocr_health = False
                    
                    if attempt < max_attempts - 1:
                        time.sleep(1.0)
                        if not self.check_ocr_health():
                            rospy.logerr("OCR引擎恢复失败")
                            break
                        time.sleep(0.5)
                
                elif attempt < max_attempts - 1:
                    time.sleep(0.5)
        
        return None
    
    def save_result_image(self, original_image, plate_text, confidence, bbox=None):
        """保存识别结果图像"""
        try:
            result_img = original_image.copy()
            
            if bbox is not None:
                x1, y1, x2, y2 = bbox
                cv2.rectangle(result_img, (x1, y1), (x2, y2), (0, 255, 0), 3)
            
            font = cv2.FONT_HERSHEY_SIMPLEX
            display_text = f"{plate_text} ({confidence:.2f})"
            
            font_scale = 1.0
            thickness = 2
            text_size = cv2.getTextSize(display_text, font, font_scale, thickness)[0]
            
            cv2.rectangle(result_img, (10, 10), (20 + text_size[0], 40), (0, 255, 0), -1)
            cv2.putText(result_img, display_text, (20, 35), 
                       font, font_scale, (0, 0, 0), thickness)
            
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            safe_text = re.sub(r'[^\w]', '', plate_text)[:10]
            filename = f"plate_{timestamp}_{safe_text}.jpg"
            filepath = os.path.join(self.save_dir, filename)
            
            if cv2.imwrite(filepath, result_img):
                rospy.loginfo(f"已保存结果图像: {filepath}")
                return filepath
            else:
                rospy.logwarn("保存图像失败")
                return None
                
        except Exception as e:
            rospy.logwarn(f"保存结果图像时出错: {e}")
            return None
    
    def handle_ocr_request(self, req):
        rospy.loginfo("收到OCR识别请求")
        
        with self.processing_lock:
            rospy.loginfo("开始处理OCR请求 (简化模式)...")
            
            try:
                cv_image = self.bridge.imgmsg_to_cv2(req.image, "bgr8")
                original_image = cv_image.copy()
                
                # 修改点2：启用新裁剪逻辑
                crop_image, bbox = self.get_center_crop(cv_image)
                
                if crop_image.size == 0:
                    rospy.logwarn("裁剪区域为空")
                    return LicensePlateOCRResponse(
                        plate_texts=[],
                        confidences=[],
                        status_message="裁剪区域无效"
                    )
                
                enhanced_crop = self.enhance_image(crop_image)
                
                temp_dir = "/tmp/ros_ocr"
                os.makedirs(temp_dir, exist_ok=True)
                temp_path = os.path.join(temp_dir, f"plate_center_{int(time.time()*1000)}.jpg")
                cv2.imwrite(temp_path, enhanced_crop)
                
                rospy.loginfo("开始OCR识别...")
                ocr_result = self.safe_ocr_process(temp_path)
                
                try:
                    os.remove(temp_path)
                except:
                    pass
                
                if ocr_result and ocr_result['success'] and ocr_result['texts']:
                    texts = ocr_result['texts']
                    scores = ocr_result['scores']
                    
                    combined_text = "".join(texts)
                    avg_confidence = sum(scores) / len(scores) if scores else 0.0
                    
                    processed_text = self.postprocess_plate(combined_text)
                    
                    if processed_text and len(processed_text) >= 2:
                        rospy.loginfo(f"识别成功: {processed_text}, 置信度: {avg_confidence:.4f}")
                        
                        saved_path = self.save_result_image(
                            original_image, processed_text, avg_confidence, bbox
                        )
                        if saved_path:
                            rospy.loginfo(f"结果图像已保存: {saved_path}")
                        
                        return LicensePlateOCRResponse(
                            plate_texts=[processed_text],
                            confidences=[avg_confidence],
                            status_message="识别成功"
                        )
                
                rospy.loginfo("未识别到有效车牌")
                return LicensePlateOCRResponse(
                    plate_texts=[],
                    confidences=[],
                    status_message="未识别到车牌"
                )
                    
            except Exception as e:
                rospy.logerr(f"OCR处理失败: {e}")
                return LicensePlateOCRResponse(
                    plate_texts=[],
                    confidences=[],
                    status_message=f"处理失败: {str(e)[:50]}"
                )
    
    def postprocess_plate(self, text):
        """后处理修正车牌"""
        if not text:
            return ""
        
        text = text.strip()
        
        remove_chars = " .·:;,'\"`~!@#$%^&*()_+-=[]{}|\\<>/?、，。！？《》（）【】"
        for char in remove_chars:
            text = text.replace(char, "")
        
        text = text.upper()
        
        patterns = [
            r'[\u4e00-\u9fff][A-Z0-9]{5,7}',
            r'[A-Z0-9]{5,8}',
        ]
        
        for pattern in patterns:
            match = re.search(pattern, text)
            if match:
                return match.group(0)
        
        return text if len(text) >= 2 else ""
    
    def shutdown(self):
        """优雅关闭"""
        rospy.loginfo("正在关闭OCR服务...")

def main():
    server = LicensePlateOCRServer()
    rospy.on_shutdown(server.shutdown)
    rospy.spin()

if __name__ == "__main__":
    main()