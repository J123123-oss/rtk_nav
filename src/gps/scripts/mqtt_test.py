import paho.mqtt.client as mqtt
import ssl

# MQTT 配置
mqtt_broker = "pec81f9f.ala.cn-hangzhou.emqxsl.cn"
mqtt_port = 8883
mqtt_user = "admin"
mqtt_pass = "admin"
mqtt_topic = "device/860065077034033/upload"  # 订阅的主题
client_id = "mqttx_096d5b5b"

# TLS 配置
ca_certificate = "/etc/ssl/certs/emqxsl-ca.crt"  # CA 证书路径
client_cert = None  # 如果需要客户端证书，可以指定
client_key = None  # 如果需要客户端私钥，可以指定

# 当连接成功时调用的回调函数
def on_connect(client, userdata, flags, rc, properties=None):
    print(f"Connected with result code {rc}")
    # 连接成功后订阅主题
    if rc == 0:
        client.subscribe(mqtt_topic)
        print(f"Subscribed to topic {mqtt_topic},message: {properties}")

# 当收到消息时调用的回调函数
def on_message(client, userdata, msg):
    print(f"Message received on topic {msg.topic}: {msg.payload.decode()}")

# 创建 MQTT 客户端实例
client =mqtt.Client(client_id="robot_bridge",
                                    callback_api_version=mqtt.CallbackAPIVersion.VERSION2 )
    # client = mqtt.Client(client_id="robot_bridge", callback_api_version="2.0")

# 设置用户名和密码
client.username_pw_set(mqtt_user, mqtt_pass)

# 设置 TLS 加密
client.tls_set(ca_certs=ca_certificate, certfile=client_cert, keyfile=client_key, tls_version=ssl.PROTOCOL_TLSv1_2)

# 设置回调函数
client.on_connect = on_connect
client.on_message = on_message

# 连接到 MQTT 代理
client.connect(mqtt_broker, mqtt_port, 60)

# 循环等待消息
client.loop_forever()

