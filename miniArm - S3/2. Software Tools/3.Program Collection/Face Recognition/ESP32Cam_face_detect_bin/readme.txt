/*
 * 2024/01/26  CuZn
*/

ESP32Cam的 人脸识别

下载
"0x0000" : "face_IIC_slave_vf0.bin"或"face_IIC_slave_vf1.bin"

两个bin文件的不同处在于  摄像头的上下翻转画面
如果烧录其中一个bin文件后，人脸正视ESP32Cam时，无法识别，上下颠倒后可以识别到，则烧录另一个bin文件即可正视识别。