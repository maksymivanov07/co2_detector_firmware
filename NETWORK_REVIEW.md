# Мережевий аудит 4.1.2

Перевірено локальні HomeSpan 2.1.8, PubSubClient 2.8 та Arduino-ESP32 3.3.11.

| Шлях | Висновок | Стан |
|---|---|---|
| HTTP response | NetworkClient write може чекати 10 с і більше; watchdog теж 10 с | Обмежено неблокувальним send, 2.5 с на відповідь / 500 мс без прогресу |
| Історія/CSV | Chunked footer обходить virtual write | Відомий Content-Length; без прямих chunk-footer write |
| HTTP parser / OTA upload | Читання заголовків і тіла має окремі тайм-аути; немає загальної межі всього запиту | Залишковий ризик; stage 2, потрібен окремий тест повільного upload |
| HomeSpan | HapOut::HapStreamBuffer::flushBuffer викликає звичайний client.write; частковий результат не перевіряє | Залишковий ризик; stage 1. Не змінювали встановлену бібліотеку |
| MQTT | TCP connect налаштовано 1 с, але hostByName виконується до connect; readByte має ліміт на байт, не весь пакет; write звичайний | Залишковий ризик при ввімкненому MQTT; stage 3 |
| Wi-Fi reconnect | Коли HomeKit активний, reconnect належить HomeSpan; паралельний reconnect не додавали | Причини від'єднання та GOT_IP рахуються окремо через атоміки |
| SCD40 / Wire | Wire timeout 50 мс; stop/service очікують через машину станів | Попередні host-регресії пройдені |
| Архів/NVS/Serial/DNS | Окремі тривалості та RTC-маркери | stage 4/8/7/6 відповідно |

Діагностика: boot_id, uptime, reset reason, RTC boot count, RSSI, Wi-Fi reason,
число обривів/GOT_IP, максимальні тривалості етапів, HTTP abort count.
RTC boot count не є довічним лічильником і не гарантований після зняття живлення.
Атоміки роблять callback безпечним щодо пам'яті; набір полів не є атомарним
знімком однієї події. При швидких подіях зберігається остання причина та загальний count.

## Збір даних

```sh
python3 tools/collect_diagnostics.py --url http://192.168.86.191 --duration 900
# Пароль запитується приховано. Для USB потрібен pyserial:
python3 tools/collect_diagnostics.py --port /dev/cu.usbmodem14101 --duration 900
```

JSONL пишеться в build/diagnostics з правами 0600; існуючі файли не перезаписуються.
Зберігається лише дозволений перелік діагностичних полів, без паролів і HomeKit PIN.
HTTP працює і з 4.1.0+, USB diagnostics — з 4.1.2. Колектор виконується обмежений
час у foreground; Ctrl-C завершує його. Тайм-аут HTTP socket може завершити
поточне читання трохи пізніше загального deadline, але trickle-read не триває
необмежено. USB-порт зберігається відкритим після тайм-ауту; це важливо для
запобігання перезапускам від повторного відкриття порту.

## Валідація

- Тести actual HTTP writer: нормальна передача, stalled/trickle/disconnected peer,
  загальний deadline, наступний запит — PASS.
- 7 тестів колектора: фільтрація секретів, reboot detection, timeout/recovery,
  redirect rejection, fragmented USB JSON, збереження порту при timeout — PASS.
- C3/core 3.3.11: compile PASS, flash 1784473, static RAM 80200 байт.
- USB upload 4.1.2: hash verified.
- Перший запуск старого колектора спричинив USB resets (reason 11); виправлено
  колектор, цей запуск виключено з оцінки стабільності firmware.
- Реальний stalled-browser test: abort count 0→1, HTTP max 632 мс, boot_id
  незмінний, наступний API-запит успішний. HTML/JSON history/CSV Content-Length
  збігається з фактичним тілом. Докази — build/diagnostics.
- Спальня, 2026-09-07, 4.1.2 / TX 13 dBm: після перенесення boot_id
  4233964467 залишається незмінним; на uptime 404 с вимірювання свіжі,
  HomeKit stage max 51 мс, HTTP stage max 69 мс. Спочатку асоціація з RSSI
  -66..-73 dBm без доступності HTTP, потім явна втрата Wi-Fi; 11 подій
  disconnect, GOT_IP лише 1, остання причина 204 (handshake timeout).
  Користувач підтвердив недоступність також Apple Home. Поворот на 90 градусів
  не відновив доступ у наступному приблизно хвилинному вікні спостереження.
  Це не reboot/watchdog у зафіксованому епізоді; причина радіо/мережевого
  з'єднання ще не встановлена. Дані: bedroom-4.1.2-usb/http.jsonl.

- Наступна перестановка у спальні: новий POWERON boot_id 729827857, тому
  зміна місця змішана з перезапуском. На uptime 7..92 с: 18/18 успішних
  HTTP-запитів, RSSI -64..-60 dBm, disconnect count 0. USB без помилок;
  stage HTTP max 14 мс. Це коротке відновлення, не доказ усунення проблеми.

- Уточнення користувача: під час останньої перестановки живлення НЕ
  від'єднували. Перезапуск 16:09:40..16:09:50 UTC слід класифікувати як
  неочікуваний. Перед новим boot USB collector мав дві SerialException;
  наступний reset_reason=1 / RTC count=1. Можливий збій живлення/контакту
  або інша причина апаратного reset; доказу конкретного компонента немає.
  До uptime 157 с після цього boot Wi-Fi disconnect count залишається 0,
  HTTP доступний. Попередній висновок про відсутність reboot стосується
  лише попереднього епізоду boot_id 4233964467.

- Повернення на потрібне місце, очікуваний power cycle підтверджений користувачем; TX 13 dBm без змін. Короткий повторний запис:
  - usb: 36/36 successful; uptime 43..218; disconnects 0; boots 1; RSSI -63..-58 dBm
  - http: 36/36 successful; uptime 44..219; disconnects 0; boots 1; RSSI -63..-59 dBm
  Цей повтор не відтворив попередній обрив; потужність поки не змінювали.
