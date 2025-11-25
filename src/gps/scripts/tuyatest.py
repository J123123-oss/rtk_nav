#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import paho.mqtt.client as mqtt
import time
import hmac
import json
from hashlib import sha256
import uuid

# 设备信息
DeviceID = '2668259d644cd47368dcoz'         # 替换成自己的DeviceID
DeviceSecret = '7dMQXnRLdJarYPBA'           # 替换成自己的DeviceSecret

# MQTT服务器信息
Address = 'm1.tuyacn.com'
Port = 8883
ClientID = 'tuyalink_' + DeviceID  

# 认证信息
T = int(time.time())
UserName = f'{DeviceID}|signMethod=hmacSha256,timestamp={T},secureMode=1,accessType=1'
data_for_signature = f'deviceId={DeviceID},timestamp={T},secureMode=1,accessType=1'.encode('utf-8')
appsecret = DeviceSecret.encode('utf-8')
Password = hmac.new(appsecret, data_for_signature, digestmod=sha256).hexdigest()

# MQTT回调函数（适配v5版本，兼容v3）
def on_connect(client, userdata, flags, rc, properties=None):
    if rc == 0:
        print("Connected to MQTT broker successfully")
    else:
        print(f"Failed to connect to MQTT broker with code {rc}")

# MQTT发布回调（可选，用于确认发布结果）
def on_publish(client, userdata, mid, reason_code=None, properties=None):
    print(f"Message published successfully (mid: {mid})")

# MQTT错误回调（可选，用于捕获错误）
def on_log(client, userdata, level, buf):
    print(f"MQTT Log: {buf}")

# 创建MQTT客户端（关键修复：指定回调API版本为v5）
# callback_api_version=mqtt.CallbackAPIVersion.VERSION2 是paho-mqtt 2.0+的强制要求
client = mqtt.Client(
    client_id=ClientID,
    callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
    protocol=mqtt.MQTTv311  # 涂鸦云推荐使用MQTTv311协议
)

# 设置认证信息
client.username_pw_set(UserName, Password)

# 启用TLS加密（涂鸦云8883端口必须启用）
client.tls_set()
client.tls_insecure_set(False)  # 禁用不安全的TLS连接（生产环境推荐）

# 设置回调函数
client.on_connect = on_connect
client.on_publish = on_publish
client.on_log = on_log  # 可选，如需调试可保留

try:
    # 连接到MQTT服务器（超时时间60秒）
    print(f"Connecting to {Address}:{Port}...")
    client.connect(Address, Port, keepalive=60)

    # 启动网络循环（非阻塞模式）
    client.loop_start()
    time.sleep(2)  # 等待连接建立

    # 检查连接状态
    if client.is_connected():
        # 准备上报的数据
        current_time = int(time.time() * 1000)
        payload = json.dumps({
            "msgId": str(uuid.uuid4()),
            "time": current_time,
            "data": {
                "test_fuc": {
                    "value": "pub",
                    "time": current_time  
                }
            }
        })

        # 上报数据到云平台
        topic = f'tylink/{DeviceID}/thing/property/report'
        result = client.publish(topic, payload, qos=1)  # qos=1确保消息可靠传输
        result.wait_for_publish()  # 等待发布确认
        print(f"Published payload: {payload}")
        print(f"Published to topic: {topic}")
    else:
        print("Failed to establish MQTT connection")

    # 保持连接（如需持续上报，可在此循环中添加定时逻辑）
    while client.is_connected():
        time.sleep(1)

except KeyboardInterrupt:
    print("\nExiting...")
except Exception as e:
    print(f"Error occurred: {str(e)}")
finally:
    # 清理资源
    client.loop_stop()
    client.disconnect()
    print("Disconnected from MQTT broker")