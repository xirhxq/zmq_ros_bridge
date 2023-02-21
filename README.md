# zmq_ros_bridge

## 是什么

利用Python的ZMQ库，实现把一个设备的特定ROS消息转发到另外一个（一些）设备的ROS消息中的分布式程序。

> 目前未考虑图传，仅以数据量较小的交互来进行开发和测试。
> 

## 目前的通信

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/a3574ba4-f255-4d7f-a36f-381649e6f141/Untitled.png)

---

- 机内程序使用单机ROS网络通信；
- 机间使用数据链协议通信。

---

[https://whimsical.com/zmq-ros-bridge-Xft8mHwMHSTJHnVqjZSzjC](https://whimsical.com/zmq-ros-bridge-Xft8mHwMHSTJHnVqjZSzjC)

## 本程序的作用

充当集群内每个设备在单机ROS网络和多机数据链网络之间的数据中转站。

[https://whimsical.com/location-CPdNVTsnvmo1KLxnQ42WEH](https://whimsical.com/location-CPdNVTsnvmo1KLxnQ42WEH)

- 在每台机器上都只有一个相同的python程序。
- 需要集群共享一个无线网络（数据链 / 4G）。

## 输入和输出（V4）

[https://whimsical.com/i-o-P5ujRPmvt56t1ov5xhDwF4](https://whimsical.com/i-o-P5ujRPmvt56t1ov5xhDwF4)

- 输入：`/tx`  ROS Topic，表示要发给其余设备的消息。
- 输出：`/rx`  ROS Topic，表示其余设备发给自己的消息。

## 输入和输出（V3）

[https://whimsical.com/i-ov3-7qeEJgSbQgtCg4tX9729uP](https://whimsical.com/i-ov3-7qeEJgSbQgtCg4tX9729uP)

（以A为例）

- 输入： `/A_to_B` 和 `/A_to_C` 分别表示给B和C的消息。
- 输出： `/B_to_A` 和 `/C_to_A` 分别表示B和C发送给自己的消息。

## 输入输出格式（V4）

`/tx` 和 `/rx` 的数据类型均为 `std_msgs/Float32MultiArray` ，具体格式为： 

```bash
std_msgs/MultiArrayLayout layout
	std_msgs/MultiArrayDimension[] dim
		string label
		uint32 size
		uint32 stride
	uint32 data_offset
float32[] data
```

其中，`std_msgs/MultiArrayDimension` 的 `label` 变量可以存储字符串，我们用它来表示 `/tx` 中的目标地址或 `/rx` 中的来源地址。

例如，一个 `/tx` 话题中的一帧 `msg` 为：

[https://whimsical.com/9WWjTvSS3jC8UwJSG9pUNJ](https://whimsical.com/9WWjTvSS3jC8UwJSG9pUNJ)

代表此台机器有一个 `[3.14, 2.71, -1.57]` 的浮点数数列要发给 `B` 机。

若 `/rx` 话题中有一帧如上图所示，那么说明 `B` 机有一个同样的浮点数数列发给了本机。

## 输入输出格式（V3）

[https://whimsical.com/msgv3-9N6reRHhocAuCWRgR54odM](https://whimsical.com/msgv3-9N6reRHhocAuCWRgR54odM)

对于V3版本，由于通过消息名称（即 `/A_to_B` 或 `/A_to_C`）来判断消息目的地，所以无需设置 `label` 变量的值。接收消息亦然。

## `/tx` 话题发布示例（V4）

```python
tx_pub = rospy.Publisher('/tx', Float32MultiArray, queue_size=10)

def send_message_to(whom, data):
    msg = Float32MultiArray(data=data)
    msg.layout.dim.append(MultiArrayDimension(label=whom))
    tx_pub.publish(msg)
```

## `/rx` 话题接收示例（V4）

```python
def float_array_callback(msg):
    data = msg.data
    whom = msg.layout.dim[0].label
    print(f'Receive {data} from {whom}')
```

## `/A_to_B` 话题发布示例（V3）

```python
ros_pub = rospy.Publisher('/A_to_B', Float32MultiArray, queue_size=10)
data = [random.randint(1, 10) for _ in range(len)]
ros_pub.publish(Float32MultiArray(data=data))
```

## `B_to_A` 话题接收示例（V3）

```python
def float_array_callback(msg, whom):
    data = msg.data
    msg = struct.pack('f' * len(data), *data)
    print(f'Receive {data} from {whom}')

rospy.Subscriber(
    '/B_to_A',  
    Float32MultiArray, 
    partial(float_array_callback, whom='B')
)
```

## 功能测试实验（V4）

[https://whimsical.com/functionaltest-5u45QxTZcHYgHs7yWxDTzG](https://whimsical.com/functionaltest-5u45QxTZcHYgHs7yWxDTzG)

1. A中 `[JustRosPublisher.py](http://JustRosPublisher.py)` 发出 `/tx` 话题，其中 `data` 为长度为2的随机浮点数数组，并在 `label` 变量中包含目标名称（此例中为B）。
2. A中 `[ZmqRosBridgeV4.py](http://ZmqRosBridgeV4.py)` 收到 `/tx` 话题，根据 `label` 变量识别目标机（此例中为B），并通过 `5555` 端口发送至目标机（B）。
3. B检测到A的 `5555` 端口中有发送给B的消息，并将其转为 `/rx` 话题中的消息，并在 `label` 中包含来源名称（A）。
4. B中 `[JustRosListener.py](http://JustRosListener.py)` 收到 `/rx` 话题中的消息。

## 功能测试实验（V3）

[https://whimsical.com/functionaltestv3-Qh7jdQA1wMeGXamYTNB2mJ](https://whimsical.com/functionaltestv3-Qh7jdQA1wMeGXamYTNB2mJ)

1. A中 `[JustRosPublisher.py](http://JustRosPublisher.py)` 发出 `/A_to_B` 话题，其中 `data` 为长度为2的随机浮点数数组，并在 `label` 变量中包含目标名称（此例中为B）。
2. A中 `[ZmqRosBridgeV4.py](http://ZmqRosBridgeV4.py)` 收到 `/A_to_B` 话题，通过话题名称识别目标机（此例中为B），并通过 `5555` 端口发送至目标机（B）。
3. B检测到A的 `5555` 端口中有发送给B的消息，并将其转为 `/A_to_B` 话题中的消息。
4. B中 `[JustRosListener.py](http://JustRosListener.py)` 收到 `/A_to_B` 话题中的消息。

## 功能测试结果（V4）

### A中 `JustRosPublisher.py` 显示

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/561862aa-7306-4005-a936-07fff6e6c13c/Untitled.png)

### B中 `JustRosListener.py` 显示

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/fdfe523e-1aaf-4173-a46d-dfee37360692/Untitled.png)

- 收发功能正常，无阻塞
- 未感觉到明显延时

## 功能测试结果（V3）

### A中 `JustRosPublisher.py` 显示

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/96a657a2-98be-43e3-a856-25cff6384453/Untitled.png)

### B中 `JustRosListener.py` 显示

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/3467e9ff-e599-4c94-8251-1708da04ee52/Untitled.png)

- 功能正常，无阻塞
- 未感觉到明显延时

> 由于两机时间并不完全同步，所以此方法得到的发送时间和接收时间无法直接相减得到传送耗时。
> 

## 延时测试（Two-way Time of Flight, TwToF）

[https://whimsical.com/twtof-AzxxUHgAHHR3bUHh1PEwC2](https://whimsical.com/twtof-AzxxUHgAHHR3bUHh1PEwC2)

1. A节点发出问询（Poll），并记录时间戳 $T^A_{Poll}$
2. B节点接到问询（Poll），并记录时间戳 $T^B_{Poll}$
3. B节点等待时间 $T_{reply}$，之后发出回应（Response），并记录时间戳 $T^B_{Resp}$
4. A节点收到回应（Response），并记录时间戳 $T^A_{Resp}$

根据如下式即可计算得到延时：

$$
 T_{prop} = \frac{T_{round} - T_{reply}}2
$$

其中

$$
T_{round} = T^A_{Resp} - T^A_{Poll} \\
 T_{reply} = T^B_{Resp} - T^B_{Poll}\\
$$

## 延时测试实验（V4）

[https://whimsical.com/durationtest-SsRcBdQ6BiPfSBAqsXPFn7](https://whimsical.com/durationtest-SsRcBdQ6BiPfSBAqsXPFn7)

1. A中 `TwToFClient.py` 发出 `/tx` 消息，其中 `data` 为长度为11的浮点数数组（其中第一位为一个随机整数），并在 `label` 中包含目标地址（B）。
2. 经过A和B中 `ZmqRosBridgeV4.py` 的转发，B中 `TwToFServer.py` 收到 `/tx` 消息，并在数组最后加上自己随机得到的 T_reply，并发出 `/tx` 消息，其中 `label` 变量设置为A。
3. 经过B和A中 `ZmqRosBridgeV4.py` 的转发，A中 `TwToFClient.py` 收到 `/rx` 消息，通过数组中的数字来判断是否是自己发出的消息，并根据自己的时间戳计算得到 T_round，再从数组中获得 T_reply，最后计算出延时。

## 延时测试实验（V3）

[https://whimsical.com/durationtestv3-WW7Fs77R9mBeMkEoKEUWFx](https://whimsical.com/durationtestv3-WW7Fs77R9mBeMkEoKEUWFx)

1. A中 `TwToFClient.py` 发出 `/A_to_B` 消息，其中 `data` 为长度为11的浮点数数组（其中第一位为一个随机整数）。
2. 经过A和B中 `ZmqRosBridgeV3.py` 的转发，B中 `TwToFServer.py` 收到 `/A_to_B` 消息，并在数组最后加上自己随机得到的 T_reply，并发至 `/B_to_A` 话题中。
3. 经过B和A中 `ZmqRosBridgeV3.py` 的转发，A中 `TwToFClient.py` 收到 `/B_to_A` 消息，通过数组中的数字来判断是否是自己发出的消息，并根据自己的时间戳计算得到 T_round，再从数组中获得 T_reply，最后计算出延时。

## 延迟测试结果（V4）

### A中 `TwToFClient.py` 显示

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/690d7f09-606c-4538-85cc-21ef65cbad59/Untitled.png)

- 平均延迟42.1ms

### 同时期A `ping` B的显示

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/1fcf5bc7-9e26-42ac-b18f-44f27496cf7d/Untitled.png)

- 平均延时135.9ms

## 延迟测试结果（V3）

### A中 `TwToFClient.py` 显示

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/5b50345c-fd1f-4090-baa9-408fbe6c1021/Untitled.png)

- 平均延时99.6ms

### 同时期A `ping` B的显示

![Untitled](https://s3-us-west-2.amazonaws.com/secure.notion-static.com/61f1aeac-9ff5-41e5-bcd6-4092f1921828/Untitled.png)

- 平均延时118.7ms
