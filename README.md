# tellolink

DJI Tello drone'u Python'dan kullanmak için küçük bir kütüphane. Drone'a bağlanır, kalkış/iniş ve hareket komutlarını gönderir, drone'un gönderdiği bütün verileri (kamera görüntüsü, IMU, yükseklik, hız, konum, pil, motorlar…) okunabilir hâle getirir.

Tello'nun resmî SDK'sı yalnızca birkaç temel veri verir. Telefon uygulamasının konuştuğu iç protokolde ise drone'un kendi uçuş günlüğü de akar: 10 Hz IMU, alt kameradan optik akış ile konum ve hız, motor değerleri ve daha fazlası. `tellolink` bu protokolü konuşur. Nereden geldiği ve TelloPy'den farkı için bkz. [Hakkında](#hakkında).

## İçindekiler

1. [Kurulum](#1-kurulum)
2. [Drone'a bağlanma](#2-dronea-bağlanma)
3. [Hemen dene: klavyeyle uçur](#3-hemen-dene-klavyeyle-uçur)
4. [Kendi kodunda kullanmak](#4-kendi-kodunda-kullanmak)
5. [Veriyi okumanın iki yolu](#5-veriyi-okumanın-iki-yolu)
6. [Komutlar](#6-komutlar)
7. [Veriler](#7-veriler)
8. [Birimler, eksenler ve zaman](#8-birimler-eksenler-ve-zaman)
9. [Birden çok drone](#9-birden-çok-drone)
10. [Güvenlik](#10-güvenlik)
11. [Bilinen sınırlar](#11-bilinen-sınırlar)
12. [Hakkında](#hakkında)
13. [Lisans](#lisans)

## 1. Kurulum

Python 3.10 veya üstü gerekir.

```
pip install -r requirements.txt
```

- `av`: drone'un H.264 görüntüsünü çözer.
- `numpy`: görüntü numpy dizisi olarak verilir.
- `pygame`: yalnızca klavye arayüzü (`keyboard_control.py`) için.

`tellolink` klasörünü projenize kopyalayın; `from tellolink import Tello` ile kullanılır. Ayrıca kurulum gerekmez.

## 2. Drone'a bağlanma

1. Drone'u açın, ön ışık sarı yanıp sönmeye başlasın.
2. Bilgisayarın Wi-Fi'ından drone'un ağına bağlanın (adı `TELLO-XXXXXX` gibidir). Bu ağda internet yoktur, bu normaldir.
3. Telefondaki Tello uygulaması açıksa kapatın: drone aynı anda tek bir bağlantıyla konuşur.
4. Windows ilk çalıştırmada güvenlik duvarı izni sorarsa izin verin. Drone verileri UDP 9000 ve 6038 portlarına gönderir.

Bağlanıp bağlanamadığınızı denemenin en kolay yolu (drone kalkmaz):

```
python tellolink/example.py
```

`Connected.` ve ardından her saniye `status`, `imu`… satırları görünüyorsa her şey çalışıyordur. `Could not connect to Tello` görürseniz 2. adıma geri dönün.

## 3. Hemen dene: klavyeyle uçur

```
python tellolink/keyboard_control.py
```

Bir pencere açılır: solda drone'un kamerası (üstte pil, yükseklik ve hızlar), sağda canlı veri paneli: yaw/pitch/roll, IMU (ivme, açısal hız, hız, barometre, sıcaklık), MVO, nav konumu, ToF, pil gerilimi, motorlar ve Wi-Fi. Bir süredir yenilenmeyen veri grileşir.

| Tuş | Ne yapar |
|---|---|
| `T` | Kalkış |
| `L` | İniş |
| `W` / `S` | İleri / geri |
| `A` / `D` | Sola / sağa kay |
| `↑` / `↓` | Yüksel / alçal |
| `←` / `→` | Sola / sağa dön |
| `Boşluk` | Basılı tutulduğu sürece dur |
| `F` | Takla. `W`/`A`/`S`/`D` basılıyken o yöne (ör. `S`+`D` arka sağa), değilse öne |
| `1` / `2` | Hareket hızı (ileri/geri/yan) azalt / artır |
| `3` / `4` | Yükselme/alçalma hızı azalt / artır |
| `5` / `6` | Dönüş hızı azalt / artır |
| `I` | Veri panelini aç / kapat |
| `ESC` | İn ve çık (pencereyi kapatmak da aynı) |

Hızlar 0.1 ile 1.0 arasında 0.1 adımla değişir; başlangıçta hepsi 0.3. Başlangıç hızlarını komut satırından da verebilirsiniz:

```
python tellolink/keyboard_control.py --speed 0.4 --vertical 0.3 --turn 0.5
```

Takla için drone havada olmalı ve pil yeterli olmalı; pil düşükse drone taklayı yapmaz. Etrafta en az 2 m boşluk bırakın.

Tuş bırakılınca drone durur ve havada asılı kalır. Pencere odakta değilken (başka bir pencereye tıkladıysanız) tuşlar dikkate alınmaz, drone yine durur.

## 4. Kendi kodunda kullanmak

En basit program:

```python
import time
from tellolink import Tello

drone = Tello()             # 1. drone nesnesi
drone.connect()             # 2. bağlan (en çok 10 s bekler, olmazsa TimeoutError). Video da başlar.

time.sleep(1)               # ilk verilerin gelmesi için
print(drone.status.battery_percent, '%')    # 3. veri oku

drone.takeoff()             # 4. komut ver
time.sleep(5)
drone.set_sticks(pitch=0.3, roll=0, yaw=0, throttle=0)   # ileri git...
time.sleep(1)
drone.set_sticks(pitch=0, roll=0, yaw=0, throttle=0)     # ...ve dur
time.sleep(2)
drone.land()
time.sleep(5)

drone.close()               # 5. bağlantıyı kapat
```

Bilinmesi gerekenler:

- **`set_sticks` bir kumanda gibidir:** verdiğiniz değer, yenisini verene kadar drone'a sürekli gönderilir. `pitch=0.3` dedikten sonra drone durmaz; durdurmak için hepsini 0 yapın.
- **Değerler −1…1 arasındadır.** 0.3 sakin bir hızdır; kapalı alanda 0.5'i geçmeyin.
- **`connect()` arkada thread'ler başlatır.** Program bitmeden `close()` çağırın. `close()` drone'u indirmez; önce `land()` deyin.

## 5. Veriyi okumanın iki yolu

### a) O an ne varsa oku

Kütüphane arkada drone'dan gelen her şeyi dinler ve **son değerleri** nesnenin içine yazar: `drone.imu`, `drone.status`, `drone.nav`… Siz istediğiniz an okursunuz. Henüz hiç gelmemiş veri `None`'dır.

```python
if drone.imu is not None:
    print(drone.imu.vel_x)
```

Ekranda değer göstermek, pil kontrol etmek gibi işler için bu yeterli. Ama iki okuma arasında gelen kayıtları görmezsiniz: IMU saniyede 10 kez gelir, siz saniyede bir okursanız 9'unu kaçırırsınız.

### b) Olaya abone ol (hiçbir kayıt kaçmaz)

Her veri türünün bir **olayı** vardır. Bir fonksiyonu olaya bağlarsanız, o türden her yeni kayıt geldiğinde fonksiyonunuz çağrılır:

```python
def on_imu(drone, data):            # data: yeni gelen Imu kaydı; drone: kaydın geldiği Tello
    print(data.tick, data.vel_x)

drone.subscribe(drone.EVENT_IMU, on_imu)     # connect'ten ÖNCE abone olun, ilk kayıtlar kaçmasın
drone.connect()
```

Kayıt önce `drone.imu`'ya yazılır, sonra olay tetiklenir. Bu yüzden fonksiyonun içinde `drone.imu`, `drone.nav` gibi alanlar da güncel olur. Bırakmak için `drone.unsubscribe(drone.EVENT_IMU, on_imu)`.

| Olay | Kaynak | `data` |
|---|---|---|
| `EVENT_CONNECTED`, `EVENT_DISCONNECTED` | bağlantı (2 s paket gelmezse kopmuş sayılır, kütüphane yeniden bağlanmayı dener) | `None` |
| `EVENT_STATUS` | uçuş durumu paketi | `Status` |
| `EVENT_WIFI` | Wi-Fi paketi | `Wifi` |
| `EVENT_IMU`, `EVENT_GYRO`, `EVENT_MVO`, `EVENT_TOF`, `EVENT_NAV`, `EVENT_OSD`, `EVENT_STICKS`, `EVENT_ATTITUDE_TARGET`, `EVENT_MOTORS`, `EVENT_BATTERY` | günlük paketi; her kayıt kendi olayıyla | aynı adlı alanın kaydı |
| `EVENT_UNSOLVED` | günlük paketi; anlamı çözülmemiş kayıtlar | `UnsolvedRecord` (`record_id`, ham `payload`) |
| `EVENT_FRAME` | video | `Frame` |

**Farklı hızlar:** Drone günlük verisini paketler hâlinde gönderir; bir pakette farklı hızlardaki kayıtlar birlikte gelir (IMU 10 Hz, gyro 20 Hz, ToF 5 Hz…). Her kayıt kendi olayını tetiklediği için bununla uğraşmanız gerekmez: ToF gelmediyse ToF olayı çalışmaz.

**Önemli — thread:** Olay fonksiyonlarınız kütüphanenin arka plan thread'inde çalışır (`EVENT_FRAME` video thread'inde, diğerleri veri thread'inde). Fonksiyon uzun sürerse sonraki paketler bekler. Kısa işler (bir değeri saklamak, yazdırmak, ROS'a `publish`) sorun değildir. Uzun iş için veriyi bir kuyruğa atıp kendi döngünüzde işleyin:

```python
import queue
q = queue.Queue()
drone.subscribe(drone.EVENT_IMU, lambda drone, data: q.put(data))
drone.connect()

while True:
    imu = q.get()           # yeni kayıt gelene kadar bekler; burası sizin thread'iniz
    ...                     # istediğiniz kadar uzun iş
```

Olay fonksiyonunda bir hata çıkarsa kütüphane durmaz; hata günlüğe (`logging`) yazılır.

## 6. Komutlar

| Fonksiyon | Ne yapar | Birim / aralık |
|---|---|---|
| `Tello(local_ip=None)` | Nesne oluşturur. Birden çok drone için o drone'un adaptörünün adresi verilir | IP metni |
| `connect(timeout=10)` | Bağlanır, videoyu başlatır. Olmazsa `TimeoutError` | s |
| `close()` | Bağlantıyı kapatır (drone'u indirmez) | — |
| `connected` | Bağlı mı | bool |
| `takeoff()` / `land()` | Kalkış / iniş | — |
| `flip(direction='forward')` | Takla: `'forward'`, `'back'`, `'left'`, `'right'`, `'forward_left'`, `'forward_right'`, `'back_left'`, `'back_right'` | drone havada, pil yeterli |
| `emergency()` | Motorları hemen durdurur; havadaysa düşer | — |
| `set_sticks(pitch, roll, yaw, throttle)` | Çubuklar; yeni değer verilene kadar sürekli gönderilir | −1…1 (pitch + ileri, roll + sağa, yaw + saat yönü, throttle + yukarı) |
| `set_altitude_limit(m)` | Yükseklik sınırı | m (tam sayı) |
| `set_low_battery_threshold(percent)` | Düşük pil uyarı eşiği | % |
| `set_wifi_name(name)` / `set_wifi_password(password)` | Wi-Fi adı / şifresi; drone yeniden açılınca geçerli olur | ASCII; şifre en çok 19 karakter |

## 7. Veriler

Her veri bir Python nesnesidir (dataclass); alanlarına `drone.imu.vel_x` gibi erişilir. "Ne zaman" sütunu verinin hangi durumda geldiğini söyler: örneğin gyro yalnızca havada, motorlar yalnızca motorlar dönerken gelir.

| Alan | Hz | Ne zaman | Alanlar |
|---|---|---|---|
| `drone.status` | 10 | hep | `height` (float, m), `vel_x`, `vel_y`, `vel_z` (float, m/s), `fly_time` (float, s), `battery_percent` (int, %), `battery_voltage` (float, V), `fly_mode` (int: 6 sabit, 11 kalkış, 12 iniş), `em_sky` (bool, havada), `em_open` (bool, motorlar dönüyor), `battery_low`, `battery_lower` (bool); diğer durum bitleri (`em_ground`, `imu_state`, `wind_state`…) protocol.py'deki Status sınıfında |
| `drone.imu` | 10 | hep | `acc_x/y/z` (float, g; yerde dururken z ≈ −1), `gyro_x/y/z` (float, açısal hız, birimi doğrulanmadı), `q0`–`q3` (float, kuaterniyon, q0 = w), `lin_acc_x/y/z` (float, m/s²), `vel_x/y/z` (float, m/s), `baro_raw`, `baro` (float, m), `gyro_bias_x/y/z` (float), `temperature` (float, °C) |
| `drone.gyro` | 20 | sadece havada | `stages` (3 × (x, y, z) float) |
| `drone.mvo` | 5 | hep | `vel_x/y/z` (float, m/s), `pos_x/y/z` (float, m; kalkışta sıçrar), `height` (float, m), `flags` (int) |
| `drone.tof` | 5 | hep | `distance` (float, m; yerde 0.1), `flag` (int), `counter` (int) |
| `drone.nav` | 10 | hep | `vel_x/y/z` (float, m/s), `pos_x/y/z` (float, m; kalkışta sıçramaz), `tof_vel` (float, m/s), `height` (float, m), `flags`, `counter` (int) |
| `drone.osd` | ~1.7 | hep | `height` (float, m), `vel_x/y/z` (float, m/s), `pitch`, `roll`, `yaw` (float, derece), `fly_mode` (int), `tof_height` (float, m), `fly_time` (float, s), `yaw_rate` (float, derece/s) |
| `drone.sticks` | 5 | hep | `pitch`, `roll`, `yaw`, `throttle` (float, drone'un gördüğü çubuk), `fly_mode`, `motors_on` (int) |
| `drone.attitude_target` | 2.5 | motorlar çalışırken | `target_yaw`, `target_roll`, `target_pitch`, `yaw`, `roll`, `pitch` (float, radyan) |
| `drone.motors` | 10 | motorlar çalışırken (yerde rölantide ~24 %, havada 55–65 %) | `pwm` (4 float, %; ön sağ, ön sol, arka sol, arka sağ) |
| `drone.battery` | 5 | hep | `voltage` (float, V) |
| `drone.wifi` | 1 | hep | `strength` (int), `disturb` (int ya da None) |
| `drone.frame` | ~30 | video akarken | `image` (numpy, yükseklik × genişlik × 3, BGR), `data` (bytes, ham H.264), `keyframe` (bool) |
| `drone.unsolved[record_id]` | — | kayda göre | anlamı çözülmemiş kayıtlar (ör. 1200, 1300, 10085): `record_id` (int), `payload` (bytes, ham) |

Her kaydın ortak iki alanı daha vardır: `recv_time` ve `tick` (bkz. 8. bölüm).

## 8. Birimler, eksenler ve zaman

- **Birimler:** metre, m/s, m/s², radyan (`osd` açıları derece), volt, °C, saniye. Drone'un gönderdiği değerler yalnızca ölçeklenir (mm → m gibi); başka işlem yapılmaz.
- **Hız ve konum dünya eksenindedir, drone'a göre değil.** `status`, `imu`, `nav`, `mvo` ve `osd`'deki `vel_*`, `pos_*` ve `imu.lin_acc_*` alanlarında x yaw = 0° yönü, y yaw = +90° yönü, z aşağıdır (yükselirken `vel_z` negatif). Drone 90° dönükken ileri gittiğinde hız `vel_y`'de görünür. Yaw'ın sıfır yönü drone'un kendi seçimidir (pusula yok); bir gerçek uçuşta açılıştan sonra 80° gösterdi.
- **Ham `imu.acc_*` ve `imu.gyro_*` gövdeye göredir:** x ileri, y sağa, z aşağı.
- **Gövdeye göre hız** (ileri / sağa) için yaw ile döndürün:

  ```python
  import math
  w, x, y, z = imu.q0, imu.q1, imu.q2, imu.q3
  yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))       # radyan
  ileri = math.cos(yaw) * imu.vel_x + math.sin(yaw) * imu.vel_y
  saga = -math.sin(yaw) * imu.vel_x + math.cos(yaw) * imu.vel_y
  ```
- **Açılar:** roll sağ taraf aşağı +, pitch burun yukarı +, yaw yukarıdan bakınca saat yönü +. (ROS'ta y sola, z yukarıdır; ROS'a basarken çevirmeyi kendiniz yapın.)
- **`recv_time`:** paketin bilgisayara ulaştığı an (`time.monotonic()`, saniye). Wi-Fi gecikmesi yüzünden ölçüm anından biraz sonradır ve titrektir.
- **`tick`:** drone'un kendi saat sayacı (~2 344 062 Hz). Ölçümün drone'daki gerçek anıdır; iki kayıt arasındaki süre `(tick2 - tick1) / 2344062` saniyedir. Yalnızca günlük kayıtlarında vardır; durum, Wi-Fi ve görüntüde `None`'dır.
- **Görüntünün** tick'i yoktur; karenin `recv_time`'ı, son parçasının bilgisayara vardığı andır.

## 9. Birden çok drone

Her drone kendi Wi-Fi ağını açar ve hepsinin adresi aynıdır (`192.168.10.1`). Bu yüzden her drone için bilgisayarda ayrı bir Wi-Fi adaptörü gerekir; her adaptör bir drone'un ağına bağlanır. Her drone için ayrı bir nesne oluşturup o adaptörün bilgisayardaki IP adresini verin:

```python
d1 = Tello(local_ip='192.168.10.2')     # 1. adaptörün adresi
d2 = Tello(local_ip='192.168.10.3')     # 2. adaptörün adresi
for d in (d1, d2):
    d.subscribe(d.EVENT_IMU, on_imu)    # aynı fonksiyon; ilk parametresi hangi drone olduğunu söyler
    d.connect()
```

Adaptörün adresini Windows'ta `ipconfig`, Linux'ta `ip addr` ile görebilirsiniz. Bu kısım henüz birden çok adaptörle denenmedi; Linux'ta ek ayar gerekebilir.

## 10. Güvenlik

- İlk denemeleri geniş ve boş bir alanda, düşük hızla (0.2–0.3) yapın.
- Programınız çökerse drone son verdiğiniz çubuk değeriyle gitmeye devam edebilir. Kodu `try/finally` içine alıp `finally`'de `set_sticks(0, 0, 0, 0)` ve `land()` çağırın.
- `emergency()` motorları anında durdurur: drone havadaysa **düşer**. Yalnızca gerçek acil durumda kullanın.
- Pil azalınca `drone.status.battery_low` `True` olur; inin.

## 11. Bilinen sınırlar

- **Az denendi.** Gerçek drone ile bir yer + eksen uçuşu yapıldı (bağlantı, görüntü, bütün veriler ve her eksen doğrulandı). Takla, Wi-Fi ayarları ve birden çok drone gerçek drone ile henüz denenmedi.
- Zaman düzeltmesi yapılmaz; `recv_time` ve `tick` ham olarak verilir.
- Kayıplı kareler atlanır; Wi-Fi'da paket kaybı olursa görüntü bir sonraki anahtar kareye kadar donabilir.
- Işık seviyesi verisi okunmaz (bu firmware'de hep 0 geliyordu).
- Bazı günlük kayıtlarının anlamı bilinmiyor; bunlar `EVENT_UNSOLVED` / `drone.unsolved` ile ham hâliyle verilir.

Testler (drone gerekmez): `python -m unittest discover -s tellolink/tests -t .`. Testler, geliştirme sırasında alınan uçuş kayıtlarını ve karşılaştırma için TelloPy'yi kullanır; kayıtlar bu repoda yoktur.

## Hakkında

`tellolink`, [TelloPy](https://github.com/hanyazou/TelloPy)'nin çözümleme yolundan yola çıkılarak yazıldı. Bir kapalı alan keşfi projesinde TelloPy ile Tello'nun uçuş günlüğünü okurken bazı hatalar ve eksikler bulduk. Bunları TelloPy'de düzeltirken, yalnızca ihtiyaç duyulan kısımları kapsayan daha sade ve anlaşılır bir paketin daha kullanışlı olacağına karar verdik.

**Kullandığımız TelloPy sürümünde bulunan ve burada olmayan sorunlar** (kod okunarak ve gerçek uçuş kayıtlarıyla bulundu):

- 16 bitlik işaretli sayılar 256'nın üstünde bozuluyordu; yükseklik, hız, uçuş süresi ve pil gerilimi yanlış okunuyordu.
- İki UDP paketine bölünen günlük kayıtları kayboluyor, ardından gelen paketin tamamı da atılıyordu (paketlerin ~%2'si).
- Paket ve kayıt CRC'leri hiç kontrol edilmiyordu.
- Drone'a gönderilen paketlerdeki saat, programın açıldığı anda donup kalıyordu.
- Drone'un saat isteklerine yanıt verilmiyordu.
- MVO (optik akış) hızı 10 kat büyük çıkıyordu.
- Bağlantı koptuktan sonra görüntü geri gelmiyordu.
- Bazı alan adları yanıltıcıydı: örneğin pil gerilimi `drone_fly_time_left` adıyla geliyordu.

**tellolink'in yaptıkları:**

- Uçuş günlüğünden 10 kayıt türünü çözer. TelloPy yalnızca birkaçını (IMU, MVO) çözüyordu; motorlar, pil gerilimi, drone'un kendi konum kestirimi, çubuklar gibi kayıtlar gerçek uçuşlarla çözüldü. Alan adları, birimler ve eksenler ölçümlerle doğrulandı.
- Yalnızca bağlantı, uçuş komutları, temel ayarlar, telemetri ve görüntü vardır. Kayıt araçları, örnek programlar gibi kapsam dışı kısımlar alınmadı.
- Her veri hem nesnenin içinde son değer olarak durur, hem de kendi olayıyla gelir.
- Görüntü kütüphanenin içinde çözülür ve her kare kendi varış zamanını taşır.
- Bağımlılık olarak yalnızca `av` ve `numpy` gerekir.

**Teşekkür:** Bu kütüphane, TelloPy'yi geliştiren [Hanyazou](https://github.com/hanyazou)'ya ve TelloPy'ye katkıda bulunan herkese çok şey borçlu. Tello'nun belgelenmemiş protokolünü çözüp açık kaynak olarak paylaşmasalardı bu çalışma mümkün olmazdı. Protokol bilgisinin bir kısmı ayrıca [TelloLib](https://github.com/Kragrathea/TelloLib), [SMerrony/tello](https://github.com/SMerrony/tello) ve DJI uçuş kayıtları için yazılmış DatCon projelerinden geliyor; emeği geçen herkese teşekkürler.

## Lisans

[Apache License 2.0](LICENSE). Protokol mantığı ve CRC tabloları TelloPy'den (Copyright 2018 Hanyazou, Apache 2.0) türetilmiştir; ayrıntılar [NOTICE](NOTICE) dosyasında.
