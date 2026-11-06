#!/usr/bin/env python3
# 语音播报服务：调用本机语音程序。
import rospy
import subprocess, re
from community_mission.srv import Speech, SpeechResponse


def handle_speech(req):
    text = req.text
    rospy.loginfo("正在播报: %s", text)

    try:
        # 预处理：将数字转换为单独播报的中文读法
        processed_text = convert_numbers_to_chinese_digits(text)

        # 播报处理后的文本
        cmd = ["espeak-ng", "-v", rospy.get_param("~voice", "mb-cn1"), "-s", str(rospy.get_param("~rate", 200)), processed_text]
        subprocess.run(cmd, check=True, timeout=rospy.get_param("~timeout", 30.0))

        return SpeechResponse(True)

    except Exception as e:
        rospy.logerr("播报失败: %s", e)
        return SpeechResponse(False)

def convert_numbers_to_chinese_digits(text):
    """将数字转换为单独播报的中文数字读法，并修复编码问题"""

    # ----- 新增：编码检查和修复 -----
    # 如果输入的是字节（bytes），尝试用UTF-8解码
    if isinstance(text, bytes):
        try:
            text = text.decode('utf-8')
        except UnicodeDecodeError:
            # 如果UTF-8失败，尝试系统常用编码（根据实际情况调整）
            try:
                text = text.decode('gb2312')  # 中文系统常用编码
            except UnicodeDecodeError:
                text = text.decode('latin-1', errors='replace')  # 最后兜底
                rospy.logwarn("车牌文本使用了非常见编码，已进行替换处理。")

    # 同时，确保后续字符串是Unicode
    # 如果处理过程中从其他来源（如日志、文件）读取，也可能引入编码问题
    rospy.logdebug(f"处理前文本 (类型: {type(text)}): {repr(text)}")
    # ----- 编码修复结束 -----

    # 原有的数字转换逻辑从这里开始...
    chinese_digits = {
        '0': '零', '1': '一', '2': '二', '3': '三', '4': '四',
        '5': '五', '6': '六', '7': '七', '8': '八', '9': '九',
        '.': '点'
    }

    # ... 您之前的数字转换代码保持不变 ...
    # 例如：
    def convert_match(match):
        number_str = match.group(0)
        if len(number_str) <= 10:
            result = []
            for char in number_str:
                result.append(chinese_digits.get(char, char))
            return ' '.join(result)
        return number_str

    import re
    pattern = r'-?\d+(?:\.\d+)?'
    result = re.sub(pattern, convert_match, text)

    rospy.logdebug(f"处理后文本: {repr(result)}")
    return result

def main():
    rospy.init_node('speech_server')
    rospy.Service('/speech_say', Speech, handle_speech)
    rospy.loginfo("语音服务已启动 /speech_say")
    rospy.spin()

if __name__ == "__main__":
    main()
