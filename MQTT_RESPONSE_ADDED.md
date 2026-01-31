# MQTT Command Response Feature Added

## Ngày: 2026-01-31

## Vấn đề
Code mới (`Sim7070GDevice.cpp`) chỉ nhận và xử lý lệnh từ MQTT server mà không gửi response lại, khác với code cũ (`main_old.cpp`) luôn gửi trạng thái hiện tại về server sau khi xử lý lệnh.

## Giải pháp
Đã thêm chức năng gửi response tự động sau khi nhận và xử lý lệnh MQTT.

## Thay đổi trong `Sim7070GDevice.cpp`

### 1. Response khi lệnh ngoài time window (dòng 464-471)
```cpp
if (dsec > WINDOW_SEC)
{
  DEBUG_PRINTLN(F("[TIME] Command outside of time window (>30 s). Ignored."));
  // Send response to server when command is rejected
  extern bool publishTelemetry();
  DEBUG_PRINTLN(F("[TIME] Sending rejection response to server..."));
  publishTelemetry();
  return;
}
```

### 2. Response sau khi xử lý lệnh (dòng 500-572)
Hàm `handleMqttCommandJson()` đã được cập nhật:

```cpp
void Sim7070GDevice::handleMqttCommandJson(const char *plain)
{
  bool shouldPublishResponse = false;

  // Xử lý lệnh X (ON/OFF)
  if (xVal == 1)
  {
    if (g_pumpDevice->isProtectionActive())
    {
      // Lệnh bị từ chối do protection
      shouldPublishResponse = true;
    }
    else
    {
      // Bật máy bơm
      g_pumpDevice->requestTurnOn();
      shouldPublishResponse = true;
    }
  }
  else if (xVal == 0)
  {
    // Tắt máy bơm
    g_pumpDevice->requestTurnOff();
    shouldPublishResponse = true;
  }

  // Gửi response về server
  if (shouldPublishResponse)
  {
    extern bool publishTelemetry();
    DEBUG_PRINTLN(F("[ACT] Sending response to server..."));
    publishTelemetry();
  }
}
```

## Các trường hợp gửi response

1. **Lệnh ON (X=1)**:
   - Nếu đang trong protection → Gửi response (lệnh bị từ chối)
   - Nếu không protection → Bật máy bơm và gửi response

2. **Lệnh OFF (X=0)**:
   - Tắt máy bơm và gửi response

3. **Lệnh ngoài time window (>30s)**:
   - Từ chối lệnh và gửi response

## Nội dung response
Response chứa trạng thái hiện tại của thiết bị:
- `X`: Trạng thái máy bơm (1=ON, 0=OFF)
- `P`: Trạng thái protection (1=active, 0=inactive)
- `B`: Phần trăm pin
- `CSQ`: Chất lượng tín hiệu
- `V1, V2, V3`: Điện áp AC
- `TCPU`: Nhiệt độ CPU
- `FLOW`: Lưu lượng nước hiện tại
- `TOT`: Tổng lưu lượng nước
- `PWR`: Nguồn điện chính (1=có, 0=không)
- `AT_UTC`: Timestamp UTC
- `RSIM, RGPRS, RMQTT`: Số lần restart/reconnect

## Lợi ích
1. **Server nhận được feedback**: Biết được lệnh đã được thực thi hay bị từ chối
2. **Giống code cũ**: Giữ nguyên behavior của hệ thống cũ
3. **Real-time status**: Server luôn cập nhật trạng thái mới nhất sau mỗi lệnh
4. **Debugging dễ hơn**: Dễ dàng theo dõi và debug qua response

## Testing
Để test tính năng này:
1. Gửi lệnh ON/OFF từ MQTT client
2. Kiểm tra serial monitor xem có log `[ACT] Sending response to server...`
3. Kiểm tra MQTT client nhận được message mới từ topic `xtr/nodes/{NODE_ID}`
4. Verify nội dung response chứa trạng thái hiện tại

## Notes
- Response được gửi qua hàm `publishTelemetry()` có sẵn trong `main.cpp`
- Response được mã hóa AES-GCM giống như telemetry thông thường
- Nếu MQTT chưa kết nối, message sẽ được enqueue và gửi khi kết nối lại
