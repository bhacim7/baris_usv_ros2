# bhacim7

# Günlük Çalışma Rehberi

Bilgisayarı açtıktan sonra ortama girmek, simülasyonu açmak, ölçüm almak ve günü kapatmak için
gereken her şey. Komutları yukarıdan aşağı sırayla çalıştır. Terminale yapıştırma kısayolu:
`Ctrl + Shift + V`. Son güncelleme: 31 Ekim 2026 (Faz 6, R1.6 sonrası).

Hızlı yol:
- Her gün: **1 → 2** (açılış, container).
- Kod değiştiyse: **3** (derleme ve testler).
- Sonra yapacağın işe göre: **5** Gölcük (TEKNOFEST, donmuş), **6** RoboBoat (şu anki iş), **7** kayıt ve analiz.

---

## 0. Önce bil: hangi komut nerede çalışır

| Satır başı | Neredesin | Burada ne yapılır |
|---|---|---|
| `baris@...` | Ana makine (Ubuntu) | `docker ...`, `git ...`, dosya düzenleme |
| `ubuntu@...:/ws$` | Container'ın içi | `ros2 ...`, `colcon build`, `pytest`, Gazebo, RViz, rapor araçları |

- `ros2: command not found` görürsen ana makinedesin; container'a gir (2. bölüm).
- `docker: command not found` görürsen container'dasın; ana makine terminaline geç.

| Yer | Ne var |
|---|---|
| `~/usv_ros2` | ROS 2 projesi (git deposu, GitHub: `bhacim7/baris_usv_ros2`) |
| `~/usv_ros2/ws` | Container'da `/ws`. Aynı klasör, iki adres |
| `~/usv_ros2/ws/src` | Paketler (container'da `/ws/src`) |
| `~/usv_ros2/ws/tools` | Rapor araçları: `run_report.py` (Gölcük koşusu), `map_report.py` (harita kalitesi) |
| `~/usv_ros2/ws/bags` | Kayıtlar, rapor resimleri, harita dosyaları. Git'e girmez |
| `~/usv_ros2/docs/ARCHITECTURE.md` | Onaylanmış mimari, kararlar, yol haritası (İngilizce) |
| `~/usv_ros2/docs/open-items.md` | Senden beklenenler, sıradaki işler |
| `~/usv_legacy` | Eski multiprocess kod. Yalnızca başvuru için; içinde değişiklik yapma |

| Paket (`ws/src/`) | Ne |
|---|---|
| `usv_interfaces` | Özel mesajlar ve servisler (`ThrusterPwm`, `NavStatus`, `DetectionArray`, `LandmarkArray`, `SetTask`...) |
| `usv_core` | ROS'suz algoritma kütüphanesi: taşınan eski kod (navigator beyni, tespit, pusula filtresi) + RoboBoat parçaları (`classes`, `world_map`). Altın testler burada |
| `usv_nodes` | Teknede de çalışacak node'lar: `navigator`, `heading_source`, `mapper` |
| `usv_sim` | Simülasyon: `sim_bridge` (Cube tarafı), `fake_perception` (ZED tarafı), `survey_drive`, parkurlar (`courses/*.yaml`), tekne modeli, `sim.launch.py` |
| `usv_gz_plugins` | Gazebo eklentisi (C++): gövdeden kaldırma kuvveti (`HullBuoyancy`) |
| `usv_learn` | Öğrenme paketi; sistemin parçası değil |

| Parkur (`usv_sim/courses/`) | Ne | Durum |
|---|---|---|
| `golcuk_2026_10_14` | 14 Ekim TEKNOFEST koşusu, videodan kurulmuş | Donmuş: yeni iş yok, yalnızca karşılaştırma |
| `roboboat_2026` | RoboBoat 2026 parkuru, Nathan Benderson Park | Şu anki iş (Faz 6) |

---

## 1. Açılış

GRUB menüsünde ok tuşlarıyla **Ubuntu**'yu seç ve Enter'a bas. Varsayılan Windows olduğu için
seçmezsen Windows açılır.

---

## 2. Container'ı başlat ve içine gir

Ana makinede bir terminal aç (`Ctrl + Alt + T`):

```bash
cd ~/usv_ros2
```

```bash
xhost +local:docker
```
Beklenen: `non-network local connections being added to access control list`.
Gazebo, RViz ve `rqt_graph` pencerelerinin açılabilmesi için gerekli; her oturum açılışında
bir kez.

```bash
docker compose up -d
```
Beklenen: `Container usv  Started` ya da `Running`.

```bash
docker compose exec usv bash
```
Beklenen: satır başı `ubuntu@...:/ws$` olur. Artık container'ın içindesin.

Paketlerin yerinde olduğunu doğrula:

```bash
ros2 pkg list | grep usv
```
Beklenen altı satır: `usv_core`, `usv_gz_plugins`, `usv_interfaces`, `usv_learn`,
`usv_nodes`, `usv_sim`. Biri eksikse 3. bölümdeki derlemeyi yap.

### Ek terminal gerektiğinde

Ana makinede yeni sekme aç (`Ctrl + Shift + T`):

```bash
cd ~/usv_ros2 && docker compose exec usv bash
```

Yeni container terminallerinde ROS 2, VRX ve kendi paketlerin kendiliğinden yüklenir;
`source` yazmana gerek yoktur. Tek istisna: başka bir terminalde yeni derleme yaptıysan, açık
kalan terminallerde `source /ws/install/setup.bash` çalıştır.

---

## 3. Derleme ve testler (container içinde)

### Derleme

```bash
cd /ws && colcon build --symlink-install && source install/setup.bash
```
Beklenen: `Summary: 6 packages finished` (uyarı olabilir, `failed` olmamalı).

Ne zaman gerekir:
- yeni dosya eklendiğinde (node, launch, config, parkur);
- `setup.py`, `package.xml`, `CMakeLists.txt` değiştiğinde;
- mesaj (`.msg`) ya da servis (`.srv`) değiştiğinde.

Ne zaman gerekmez: yalnızca mevcut bir Python ya da YAML dosyasının içi değiştiğinde
(`--symlink-install` sayesinde). Emin değilsen derle; zararı yok.

### Testler

Kodda bir şey değiştiyse commit'ten **önce** üçünü de çalıştır. Kırmızı `FAILED` varsa commit
etme. Paketlerin `test/` klasöründe stil kontrolleri (flake8, pep257) de var; onlar da geçmeli.

```bash
cd /ws/src/usv_core && python3 -m pytest test/
```
Beklenen: sonda `... passed, 1 skipped`, `failed` yok. Altın testler (`test_golden_compare`:
eski navigator ile döngü döngü eşitlik) yaklaşık 25 saniye sürer. Test eklendikçe sayı artar.

```bash
cd /ws/src/usv_nodes && python3 -m pytest test/
```
Beklenen: `8 passed`.

```bash
cd /ws/src/usv_sim && python3 -m pytest test/
```
Beklenen: `68 passed, 1 skipped` (31 Ekim). Atlanan test telif başlığı kontrolüdür; baştan kapalı.

Yalnızca bir dosyayı denemek için: `python3 -m pytest test/test_survey.py -v`.

---

## 4. Simülasyon: tek komut, ne açılır

Her şey tek bir launch ile açılır; `Ctrl + C` hepsini kapatır:

```bash
ros2 launch usv_sim sim.launch.py course:=<parkur> <seçenekler>
```

Sırayla ne olur:
1. `make_course_world.py`, parkur dosyasından dünyayı `/tmp/usv_sim_<parkur>.sdf` olarak üretir.
   Dünya bu yüzden hiçbir zaman parkurdan eski olamaz.
2. Gazebo, `ros_gz_bridge`, `sim_bridge` (Cube: itici, dümen, röle, GPS, hız, FC pruvası),
   `fake_perception` (ZED: pruva, yalpa, tespitler), `heading_source` (`/usv/heading`) ve
   `mapper` (dünya haritası, her zaman açık) açılır.
3. Seçeneğe göre: `navigator`, RViz, `survey_drive`, `ros2 bag record`.

Bir node çökerse 2 saniye sonra kendiliğinden yeniden başlar (teknedeki gibi).

| Seçenek | Varsayılan | Ne yapar |
|---|---|---|
| `course:=` | `golcuk_2026_10_14` | Parkur: `golcuk_2026_10_14` ya da `roboboat_2026` |
| `gui:=false` | `true` | Gazebo penceresi açılmaz (daha hızlı) |
| `paused:=true` | `false` | Duraklatılmış başlar; sol alttaki ▶'e bas |
| `rviz:=true` | `false` | RViz dünya haritasıyla açılır |
| `navigator:=false` | `true` | Navigator açılmaz; elle sürmek için (6.3) |
| `auto:=true` | `false` | Navigator OTOMATİK modda başlar (Gölcük) |
| `start_task:=...` | parkurun kendi görevi | Navigator başlangıç görevi, ör. `TASK1_STATE_ENTER` |
| `survey:=true` | `false` | Navigator yok; `survey_drive` parkurun ölçüm rotasını sürer (RoboBoat) |
| `seed:=N` | yok (her koşu farklı) | Algılayıcı gürültüsü her koşuda aynı (iki koşuyu karşılaştırmak için) |
| `true_distance:=false` | YAML (`true`) | RoboBoat tespitleri: derinlik mesafe sayılır (eski hal, D2 kapalı) |
| `from_camera:=false` | YAML (`true`) | RoboBoat tespitleri: antenden konumlanır (eski hal, D1 kapalı) |
| `record:=<ad>` | kayıt yok | `/ws/bags/<ad>` klasörüne kayıt. Ad daha önce kullanılmamış olmalı |
| `compass_offset_deg:=8.0` | YAML | ZED pusulası 8° fazla okur (deney) |
| `detect_prob:=0.5` | YAML | YOLO bir nesneyi karelerin yarısında bulur (deney) |
| `max_range_m:=10.0` | YAML | 10 m'den uzağı görmez (deney) |
| `course_east:= course_north:= course_heading:=` | parkur dosyasındaki | Parkuru Gazebo dünyasında başka yere koy (üçü birlikte) |

Gazebo'da tekneyi bulmak: sol üstteki **Entity Tree**'de `usv_boat`'a sağ tık → **Follow**.
Uzaktaki bir nesneye gitmek: Entity Tree'de sağ tık → **Move to**.

---

## 5. Gölcük parkuru (TEKNOFEST, donmuş)

14 Ekim koşusunun videodan kurulan parkuru. Faz 5'te navigator burada eski kodla
karşılaştırıldı. TEKNOFEST için yeni iş yapmıyoruz; bu bölüm karşılaştırma ve tekrar koşmak için.

### 5.1 Manuelde başlat, sonra otomatiğe al (teknedeki gibi)

```bash
ros2 launch usv_sim sim.launch.py
```
Beklenen: Gazebo açılır, tekne Görev 2 girişinin önünde durur. Navigator **MANUEL** modda,
parkurun görevi `TASK2_START` ile başlar ve saniyede 25 kez 1500 yayınlar (tekne durur).

Ayrı bir container terminalinde otomatiğe al (yer istasyonunun "otonom" komutunun karşılığı):

```bash
ros2 service call /usv/set_manual std_srvs/srv/SetBool "{data: false}"
```
Beklenen: `success=True`; tekne kalkış kontrolünden (GPS ve pruva var mı) geçince sürmeye başlar.

Görev durumu:

```bash
ros2 topic echo /usv/nav_status --once
```

### 5.2 Doğrudan otomatik başlat

```bash
ros2 launch usv_sim sim.launch.py auto:=true
```
Navigator OTOMATİK modda açılır, servis çağrısı gerekmez. Görev 1'den başlatmak için:

```bash
ros2 launch usv_sim sim.launch.py auto:=true start_task:=TASK1_STATE_ENTER
```
Beklenen: Görev 1 kapıları → Görev 2 koridoru → Görev 3 → `FINISHED`.

### 5.3 Yer istasyonu komutlarının karşılıkları

Manuele dön:

```bash
ros2 service call /usv/set_manual std_srvs/srv/SetBool "{data: true}"
```

Görev seç:

```bash
ros2 service call /usv/set_task usv_interfaces/srv/SetTask "{task: TASK2_START}"
```

Acil durum (navigator kilitlenir, aynı döngüde nötr):

```bash
ros2 service call /usv/navigator/emergency_stop std_srvs/srv/Trigger
```

Gerçek acil durumda Cube tarafı da çağrılır (iticiler 1500, röle kapalı):

```bash
ros2 service call /usv/emergency_stop std_srvs/srv/Trigger
```

Acil durumdan çıkış (ARCHITECTURE §9): önce röleyi aç, sonra görevi yeniden seç. Mod korunur.

```bash
ros2 service call /usv/set_relay std_srvs/srv/SetBool "{data: true}"
```

```bash
ros2 service call /usv/set_task usv_interfaces/srv/SetTask "{task: TASK2_START}"
```

### 5.4 Deney koşusu ve rapor

Bir seçenekle koşup kaydet, sonra rapor al (Faz 5.6'daki deneyler böyle yapıldı):

```bash
ros2 launch usv_sim sim.launch.py auto:=true record:=exp_compass8 compass_offset_deg:=8.0
```
`FINISHED` görünce `Ctrl + C`. Sonra:

```bash
python3 /ws/tools/run_report.py /ws/bags/exp_compass8 --reference /ws/tools/golcuk_2026_10_14_reference.yaml --plot /ws/bags/exp_compass8.png
```
Beklenen: görev zaman çizelgesi, her şamandıraya en yakın geçiş (gerçek gövdeden; `+` = boşluk,
`-` = temas), hızlar, video ile karşılaştırma. Resim: `~/usv_ros2/ws/bags/exp_compass8.png`.

---

## 6. RoboBoat parkuru (şu anki iş, Faz 6)

Nathan Benderson Park'taki RoboBoat 2026 parkuru. Parkurun yeri parkur dosyasında sabit
(`placement`): tekne Gazebo dünyasında (-189,17; 947,45) noktasından batıya bakarak başlar.

Gölcük'ten farkları:
- RoboBoat görevleri navigator'a henüz eklenmedi (R2–R3). Bu yüzden RoboBoat'ta navigator ya
  kapalı (`navigator:=false`) ya da yerine `survey:=true` kullanılır. Navigator açık kalırsa
  manuel modda sürekli 1500 yayınlar ve elle sürmeyi engeller.
- `fake_perception` burada `/usv/buoys` değil `/usv/detections` yayınlar (13 sınıf, her kutu
  ölçülmüş haliyle). `mapper` bunlardan kalıcı haritayı (`/usv/map`) kurar.
- Harita sistem açıldığı anda başlar, manuel sürüşte de çalışır.

### 6.1 Parkuru ve haritayı görmek

```bash
ros2 launch usv_sim sim.launch.py course:=roboboat_2026 navigator:=false rviz:=true
```
Beklenen:
- terminalde `boat start x -189.17 y 947.45 heading 270.0 deg` ve
  `fake_perception ready: course roboboat_2026 (roboboat, 52 camera targets)`;
- Gazebo'da park, Görev 1 direkleri teknenin önünde;
- RViz'de yukarıdan bakış.

RViz'de neyi okuyacağın:
- silindirler harita izleri; soluk olan onaylanmamış (3'ten az görülmüş);
- yazılar tür ve görülme sayısı (`buoy_red x42` gibi);
- mavi ok tekne, mavi çizgi GPS izi (gerçek yol değil).

`Ctrl + C` ile kapatınca harita `/ws/bags/map_roboboat_2026.json` dosyasına yazılır
(`map saved: ... (N landmarks)`). Bu dosya **her koşuda üzerine yazılır**; saklamak istersen
kapattıktan sonra kopyala:

```bash
cp /ws/bags/map_roboboat_2026.json /ws/bags/map_roboboat_2026_<ad>.json
```

### 6.2 Ölçüm koşusu: sabit rota + harita kalitesi raporu (R1.6)

Haritanın ne kadar doğru olduğunu ölçmenin yolu bu. `survey_drive`, parkur dosyasındaki
`survey` rotasını Gazebo'nun **gerçek** konumuyla sürer. Rota yaklaşık 155 m: Görev 1
kapıları, yerinde 360° dönüş, kanal, enkaz alanı, Görev 3 kapısı, gemiler, marina ve dönüş.
GPS ve pusula yönü etkilemediği için her koşu aynı yoldan gider. `seed:=1` ile gürültü de aynı
olur; böylece bir düzeltmenin öncesi ve sonrası karşılaştırılabilir.

```bash
ros2 launch usv_sim sim.launch.py course:=roboboat_2026 survey:=true seed:=1 record:=survey_<ad> rviz:=true
```
Beklenen:
- `survey_drive: 19 steps on roboboat_2026, starting 8 s after the first true pose`;
- `survey started`, ardından adım adım `step N/19 reached (... m off)`; 3. adımda `turned +36x deg`;
- yaklaşık 220 saniye sonra `survey done in ... s; holding neutral`.

Bu mesajı görünce `Ctrl + C`. Bir adımda `timed out` yazarsa tekne bir yere takılmıştır; not al.

Rapor (aynı ya da yeni bir container terminalinde):

```bash
python3 /ws/tools/map_report.py /ws/bags/survey_<ad> --plot /ws/bags/survey_<ad>.png
```
Beklenen: birkaç saniye ile bir dakika arası sürer, üç bölüm yazar.

| Bölüm | Ne söyler |
|---|---|
| `1. sightings` | Her tespitin gerçek nesnesine göre hatası. Girdiler (GPS, pusula, mesafe, anten/kamera) tek tek gerçeğe çekilir: `only X fixed` = X düzeltilirse ne kazanılır, `only X wrong` = X tek başına ne kadar hata getirir. `all true` satırı ~0,06 m çıkmalı (aracın kendi kontrolü) |
| `2. inputs` | GPS hatası (alıcı kayması + yaş), pusula hatası dönüş hızına göre, mesafe hatası (ZED gürültüsü + geometri) uzaklığa ve ekran açısına göre |
| `3. map` | Her nesne: bulundu mu, onaylandı mı, hatası, kopyaları; hayalet izler; `TARGET` satırı |

Resimde: solda harita (halkalar gerçek nesneler, noktalar harita, çizgiler hata), sağ üstte
hatanın uzaklığa göre dağılımı, sağ altta her girdinin tek başına getirdiği hata.

Düzeltmeler (R1.7) `fake_perception.yaml`'da açık; eski hali (karşılaştırma için) launch'a
`true_distance:=false from_camera:=false` eklenerek koşulur. Raporun sonunda iki harita var:
`3a` teknenin kendi haritası (mutlak hata, hedef yok: GPS belirliyor) ve `3b` aynı tespitlerin
GPS kusursuzmuş gibi yeniden haritalanması (ölçüm zinciri, `TARGET` satırı burada).
R1.7'den önce alınan kayıtlar (ör. `survey_base`) yeni raporla okunamaz: mesajlara alan eklendi.

İlk ölçüm (`survey_base`, 31 Ekim): tespit başına ortalama 2,75 m. Paylar: mesafe 1,74 m
(derinliği mesafe sanmak), GPS 1,01 m (çoğu alıcı kayması), anten/kamera farkı 0,94 m, pusula
0,34 m (dönüşlerde 9°'ye kadar). Ayrıntı ve düzeltme sırası: `ARCHITECTURE.md` §16,
"Map quality". D2 + D1 sonrası (`survey_d1d2`, 1 Kasım): tespit başına 1,39 m, ölçüm zinciri haritası 0,28 m
(hedef geçti), mutlak harita 1,06 m (GPS kayması).

### 6.3 Elle sürmek

Navigator'sız aç (`navigator:=false`), sonra ayrı bir terminalde komut gönder. `Ctrl + C` ile
bırakınca tekne 0,5 saniye sonra durur (Cube zaman aşımı).

İleri (1680 = navigator'ın seyir PWM'i, ≈1,3 m/s):

```bash
ros2 topic pub -r 20 /usv/thrusters/pwm usv_interfaces/msg/ThrusterPwm "{rear_left: 1680, rear_right: 1680, front_left: 1680, front_right: 1680, steer: 1500}"
```

İleri giderken sağa dön (dümen 1700, sol iticiler biraz fazla):

```bash
ros2 topic pub -r 20 /usv/thrusters/pwm usv_interfaces/msg/ThrusterPwm "{rear_left: 1780, rear_right: 1580, front_left: 1730, front_right: 1630, steer: 1700}"
```

Yerinde sağa dön (sol ileri, sağ geri):

```bash
ros2 topic pub -r 20 /usv/thrusters/pwm usv_interfaces/msg/ThrusterPwm "{rear_left: 1700, rear_right: 1300, front_left: 1700, front_right: 1300, steer: 1700}"
```

### 6.4 Parkuru başka bir yere koymak

Parkur dosyasındaki yeri değiştirmeden denemek için teknenin başlangıç noktasını ve yönünü ver:

```bash
ros2 launch usv_sim sim.launch.py course:=roboboat_2026 navigator:=false course_east:=-189.17 course_north:=947.45 course_heading:=270
```
Launch yerleştirilmiş kopyayı `/tmp/usv_sim_roboboat_2026_placed.yaml` olarak yazar. O koşunun
raporunu alırken `map_report.py`'ye `--course-file /tmp/usv_sim_roboboat_2026_placed.yaml` ekle.
Yer kalıcı olacaksa parkur dosyasındaki `placement` satırı değiştirilir.

Yer seçerken: Gazebo'nun su düzlemi karanın altından da devam eder ve kara alttan görünmez.
Konumları hep **yukarıdan** bakarak (View Angle → üstten) seç. Konum okumak için: Entity
Tree'de modele tıkla → sağdaki **Component Inspector** → Pose.

---

## 7. Kayıt ve analiz

Kayıt için launch'ta `record:=<ad>` yeterli. Hangi konuların kaydedildiği `sim.launch.py`
içindeki `RECORD_TOPICS` listesinde: navigator durumu ve PWM, Gazebo'nun gerçek konumu ve IMU'su
(`/usv/sim/navsat`, `/usv/sim/imu`), GPS, pruva, hız, tespitler, harita.

| Araç | Ne için | Örnek |
|---|---|---|
| `run_report.py` | Gölcük koşusu: görevler, temaslar, hızlar, video karşılaştırması | 5.4 |
| `map_report.py` | RoboBoat haritası: hatanın kaynakları, harita kalitesi | 6.2 |

Kayıtları elle incelemek:

```bash
ros2 bag info /ws/bags/survey_base
```

```bash
ros2 bag play /ws/bags/survey_base --topics /usv/map /usv/fix
```

- Aynı adla ikinci kez kayıt alınamaz (`already exists`); her seferinde yeni ad ver.
- Kayıtlar ana makinede `~/usv_ros2/ws/bags` altında durur ve git'e girmez.
- Launch dışında elle kayıt: `ros2 bag record -o <ad> /usv/fix /usv/heading`.

---

## 8. İnceleme araçları

| Komut | Ne gösterir |
|---|---|
| `ros2 node list` | Çalışan node'lar |
| `ros2 topic list` | Var olan topic'ler |
| `ros2 topic echo /usv/fix --once` | Bir topic'ten tek mesaj |
| `ros2 topic hz /usv/detections` | Topic'in gerçek yayın hızı (kamera ≈30 Hz) |
| `ros2 topic info /usv/thrusters/pwm` | Mesaj tipi, kaç yayıncı var (birden fazlaysa çakışma) |
| `ros2 param list /mapper` | Bir node'un parametreleri |
| `ros2 param get /mapper merge_radius_m` | Tek parametrenin değeri |
| `ros2 service list` | Var olan servisler |
| `rqt_graph` | Node ve topic'lerin canlı şeması |

---

## 9. Tekne modelini tek başına denemek (isteğe bağlı)

Model ya da fizik değiştiğinde, ROS olmadan yalnızca Gazebo ile:

```bash
export GZ_SIM_RESOURCE_PATH=/ws/src/usv_sim/models:$GZ_SIM_RESOURCE_PATH
```

```bash
gz sim -v 3 /ws/src/usv_sim/worlds/nbpark.sdf
```
Dünya duraklatılmış açılır: ▶'e bas. İkinci terminalde Gazebo'nun kendi komutları:

Teknenin konumu (5 kez, 1 saniye arayla):

```bash
for i in 1 2 3 4 5; do gz model -m usv_boat -p 2>/dev/null | grep -A1 "Pose" | tail -n 1; sleep 1; done
```

Dört iticiye 10 N:

```bash
for t in front_left front_right rear_left rear_right; do gz topic -t /usv_boat/thrusters/$t/thrust -m gz.msgs.Double -p 'data: 10.0'; done
```

Ön iticilerin açısı (rad; + = tekne sola döner; sınır ±0,785):

```bash
for t in front_left front_right; do gz topic -t /usv_boat/thrusters/$t/pos -m gz.msgs.Double -p 'data: 0.5'; done
```

Hepsini durdur:

```bash
for t in front_left front_right rear_left rear_right; do gz topic -t /usv_boat/thrusters/$t/thrust -m gz.msgs.Double -p 'data: 0.0'; done
```

VRX'in hazır simülasyonu (kıyas için): `ros2 launch vrx_gz competition.launch.py world:=sydney_regatta`.
İlk açılışta modeller indiği için pencere birkaç dakika boş kalabilir.

---

## 10. Değişiklikleri kaydet (ana makinede)

Testler (3. bölüm) geçtikten sonra:

```bash
cd ~/usv_ros2 && git status
```

```bash
git add <dosyalar ya da klasörler>
```
`git add .` her şeyi ekler; ne eklediğini bil. Claude bir adım bitince hangi yolların
ekleneceğini ve commit mesajını verir.

```bash
git commit -m "Short English description of the change"
```

```bash
git push
```
Beklenen: `To github.com:bhacim7/baris_usv_ros2.git` ile biten satırlar.

---

## 11. İmajı yeniden derlemek gerektiğinde

`docker/Dockerfile` ya da `compose.yaml` değiştiğinde, ana makinede:

```bash
cd ~/usv_ros2 && docker compose build
```

```bash
docker compose up -d
```
Beklenen: `Container usv  Recreated` ve `Started`.

Container içinde `apt install` ile kurulan paketler kalıcı **değildir**; container yeniden
oluşturulunca gider. Kalıcı olması gereken her paket Dockerfile'a eklenir.

### İmaj kayıt noktası

Çalışan imaja tarihli ikinci bir ad ver (ek disk kaplamaz):

```bash
docker tag usv-ros2:dev usv-ros2:YYYY-MM-DD-aciklama
```

Yeni bir derleme ortamı bozarsa eskisine dön:

```bash
cd ~/usv_ros2 && docker tag usv-ros2:YYYY-MM-DD-aciklama usv-ros2:dev && docker compose up -d
```

| Etiket | Ne var |
|---|---|
| `usv-ros2:2026-10-22-phase3` | ROS 2 Jazzy, Gazebo Harmonic, VRX. Faz 3 sonu |

Kodun kayıt noktası imaj değil, git commit'idir (10. bölüm).

---

## 12. Kodu okumak ve düzenlemek (VS Code)

```bash
code ~/usv_ros2
```

- Dosyaları VS Code'da düzenle, `Ctrl + S` ile kaydet. Container aynı dosyaları `/ws` altında
  anında görür.
- Komutları yine container terminalinde çalıştır. VS Code'un kendi terminali (`Ctrl + J`) ana
  makinede açılır; oradan `docker compose exec usv bash` ile container'a girilir.
- VS Code `rclpy` gibi container'daki kütüphaneleri tanımadığı için bazı satırları sarı
  çizebilir; kod yine çalışır.
- VS Code'da yapıştırma `Ctrl + V`'dir (terminaldeki `Ctrl + Shift + V` değil).

---

## 13. Günün sonunda

1. Çalışan simülasyonu `Ctrl + C` ile durdur (harita dosyası bu sırada yazılır).
2. Container terminallerinde `exit` yaz.
3. Ana makinede:

```bash
cd ~/usv_ros2 && docker compose down
```
Beklenen: `Container usv  Removed`. Kod, derleme çıktıları, kayıtlar ve indirilen modeller
silinmez.

Bunu yapmadan bilgisayarı kapatsan da bir şey bozulmaz; bir sonraki açılışta 2. bölüm her şeyi
yeniden başlatır.

---

## 14. Sorun giderme

| Belirti | Sebep | Çözüm |
|---|---|---|
| `service "usv" is not running` | Container kapalı | `docker compose up -d` |
| `ros2: command not found` | Ana makinedesin | `docker compose exec usv bash` |
| `permission denied ... docker.sock` | Oturum `docker` grubunu tanımıyor | Bilgisayarı yeniden başlat; acilse o terminalde `newgrp docker` |
| Gazebo, RViz ya da `rqt_graph` penceresi açılmıyor | Ekran izni verilmemiş | Ana makinede `xhost +local:docker` |
| `Package 'usv_...' not found` | Terminal yeni derlemeyi tanımıyor | `source /ws/install/setup.bash` |
| Yeni node ya da launch seçeneği bulunamıyor | Yeni dosya eklendi ama derlenmedi | `cd /ws && colcon build --symlink-install` |
| Kayıt başlamıyor: `already exists` | Aynı `record:=` adı daha önce kullanıldı | Yeni ad ver |
| Elle `topic pub` ile tekne kıpırdamıyor | Navigator da açık, manuel modda 1500 yayınlıyor | `navigator:=false` ile aç (`ros2 topic info /usv/thrusters/pwm` iki yayıncı gösterir) |
| RoboBoat'ta navigator hiçbir şey yapmıyor | RoboBoat görevleri henüz yok, `/usv/buoys` yayınlanmıyor | Beklenen. `navigator:=false` ya da `survey:=true` |
| Harita dosyası önceki koşununki gibi değil | `map_<parkur>.json` her koşuda üzerine yazılır | Saklamak için kopyala (6.1) |
| Parkur suda değil, karanın altında ya da hiç görünmüyor | Yer alttan bakarak seçilmiş (kara alttan görünmez) | Konumu yukarıdan bakarak seç (6.4) |
| `test_flake8` / `test_pep257` kırmızı | Stil hatası (import sırası, docstring biçimi) | Çıktıyı Claude'a gönder |
| `Driver/library version mismatch` | Sürücü güncellendi, yeniden başlatılmadı | Bilgisayarı yeniden başlat |
| Gazebo'da hiçbir şey hareket etmiyor, sağ altta `0.00 %` | Dünya duraklatılmış (`paused:=true` ya da 9. bölüm) | Sol alttaki ▶ |
| Gazebo `usv_boat` modelini bulamıyor (9. bölüm) | Model klasörü tanıtılmamış | `export GZ_SIM_RESOURCE_PATH=...` satırı |
| Suyun üstünde yeşil tel çerçeveli bir kare | Güneşin (ışık kaynağının) işareti | Zararsız |
| Uzun blok yapıştırınca ekran karışık görünüyor | Terminalin çizim hatası | Dosyayı `cat` ile doğrula |

### Ekran donarsa

1. **Caps Lock**'a bas. Işık yanıyorsa sistem çalışıyor, yalnızca görüntü donmuş.
2. Güç tuşuna **bir kez kısaca** bas ve **90 saniye dokunma**. Kendiliğinden kapanırsa sistem canlıydı.
3. Kapanmazsa güç tuşunu basılı tut.
4. Açılınca şu çıktıyı Claude'a gönder:

```bash
journalctl -b -1 -n 60 --no-pager
```

13 Ekim 2026'da Intel ekran kartının PSR özelliği kapatıldı (`i915.enable_psr=0`). Hâlâ kapalı
olduğunu doğrulamak için:

```bash
sudo sh -c 'cat /sys/kernel/debug/dri/*/i915_edp_psr_status 2>/dev/null' | grep "PSR mode"
```
Beklenen: `PSR mode: disabled`

---

## 15. Ara sıra kontrol

Pil şarj sınırı (%80):

```bash
cat /sys/class/power_supply/BAT0/charge_control_end_threshold
```
Beklenen: `80`

Disk durumu:

```bash
df -h /
```

Kayıtların kapladığı yer (eski deney kayıtları silinebilir):

```bash
du -sh ~/usv_ros2/ws/bags/*
```

Docker'ın kapladığı yer:

```bash
docker system df
```
