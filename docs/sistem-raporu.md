# Sistem Raporu: Ne Yaptık, Nasıl Yaptık, Neden

Son güncelleme: 1 Kasım 2026 (Faz 6, R2 bitti; sıradaki iş R3, Görev 2).

Bu belge projenin bugünkü halini baştan sona anlatır: hangi kod ne iş yapar, veriler
sensörden itici motorlarına nasıl akar, tekne nasıl karar verir, PWM'ler nasıl hesaplanır,
haritalar neye göre çizilir, hangi ölçümleri yaptık ve ne bulduk, hangi komut ne işe yarar.

Diğer belgelerle ilişkisi:

| Belge | İçerik | Ne zaman bakılır |
|---|---|---|
| `docs/sistem-raporu.md` (bu) | Bütün sistemin açıklaması, ölçüm sonuçları | Sistemi anlamak için |
| `docs/daily-startup.md` | Komut komut günlük kullanım | Bilgisayarı açınca, bir şey çalıştırırken |
| `docs/ARCHITECTURE.md` | Onaylanmış mimari, kararlar, yol haritası (İngilizce) | Bir tasarım kararının sebebini ararken |
| `docs/open-items.md` | Senden beklenenler, sıradaki işler | Ne kaldı diye bakarken |

---

## 1. Büyük resim

**Amaç:** Eski çok süreçli (multiprocess) tekne kodunu (`~/usv_legacy`, commit `5df3f5b`)
ROS 2 Jazzy'ye taşımak. Önce Docker içinde Gazebo + VRX simülasyonunda, sonra Jetson AGX Thor
üzerinde gerçek teknede çalıştırmak. Şu an RoboBoat 2026 görevlerini simülasyonda geliştiriyoruz,
çünkü tekne bir süre suya inemiyor.

**Fazlar:**

| Faz | Ne | Durum |
|---|---|---|
| 3 | Mimari belgesi | Bitti |
| 4 | Mesajlar, ROS'suz çekirdek kütüphane, tekne modeli, simülasyon köprüleri (`sim_bridge`, `fake_perception`) | Bitti (30 Ekim) |
| 5 | Navigator'ı (eski `nav_worker`) taşımak, eskisiyle döngü döngü aynı olduğunu göstermek | Bitti (1 Kasım) |
| 6 | RoboBoat simülasyonda: R1 parkur + algılama + harita, R2 navigator'ın RoboBoat beyni, R3 görevler, R4 raporlama, R5 eski kodun düzeltmeleri | R1 ve R2 bitti, sırada R3 |
| 7 | Tekne: Thor imajı, `fc_bridge`, gerçek `perception`, telemetri, kayıt, ses | Bekliyor |
| 8 | Deniz denemeleri | Bekliyor |

**Temel ilkeler** (bütün kararların arkasındaki):
- **Ölçmeden tahmin yok.** Bir değer değiştirilecekse önce ölçülür, sonra tekrar ölçülür.
- **Yazılım simülasyonu tekneden ayırt edememeli.** Aynı topic'ler, aynı birimler, aynı
  hızlar, aynı gecikmeler ve hatalar. Simülasyondaki her parça teknedeki bir parçanın yerine
  geçer.
- **TEKNOFEST kodu donmuş.** Yeni TEKNOFEST işi yok; var olan kod, altın testlerle korunuyor.
- **Önce analiz, sonra kod.** Her adımın planı kod yazılmadan sunulur, sen onaylarsın.

---

## 2. Çalışma ortamı

### 2.1 Yapı

```
Ana makine (Ubuntu, "baris@...")
 └─ Docker container "usv" ("ubuntu@...:/ws$")
     ├─ ROS 2 Jazzy, Gazebo Harmonic, VRX (Dockerfile'da kurulu)
     └─ /ws  =  ana makinedeki ~/usv_ros2/ws (aynı klasör, iki adres)
```

Kodu ana makinede (VS Code) düzenlersin, container aynı dosyaları anında görür. ROS, Gazebo,
test ve rapor komutları container'da; `git` ve `docker` komutları ana makinede çalışır.

### 2.2 Ortam komutları ve ne işe yaradıkları

| Komut | Nerede | Ne yapar, neden |
|---|---|---|
| `xhost +local:docker` | Ana makine | Container'daki programların (Gazebo, RViz) ekrana pencere açmasına izin verir. Her oturumda bir kez |
| `docker compose up -d` | Ana makine | `compose.yaml`'a göre container'ı arka planda başlatır |
| `docker compose exec usv bash` | Ana makine | Çalışan container'ın içinde bir terminal açar |
| `docker compose down` | Ana makine | Container'ı kapatır (kod ve kayıtlar silinmez) |
| `docker compose build` | Ana makine | `Dockerfile` değişince imajı yeniden kurar |
| `cd /ws && colcon build --symlink-install` | Container | Paketleri derler. `--symlink-install` Python dosyalarını kopyalamak yerine bağlar; bu yüzden var olan bir dosyayı değiştirince yeniden derlemek gerekmez, yeni dosya eklenince gerekir |
| `source install/setup.bash` | Container | Terminale yeni derlenen paketleri tanıtır |
| `rm -rf build/usv_core install/usv_core` | Container | Bir paketin derleme kalıntılarını siler. Dosya yeniden adlandırılınca eski bağlantılar kalmasın diye kullandık; kod silinmez |
| `python3 -m pytest test/` | Container, paket klasöründe | O paketin bütün testlerini çalıştırır; stil kontrolleri (flake8, pep257) de dahil |
| `git status` / `git add` / `git commit` / `git push` | Ana makine | Değişiklikleri gör, seç, kaydet, GitHub'a gönder |

---

## 3. Kodun haritası

### 3.1 Paketler

| Paket | Tür | Ne |
|---|---|---|
| `usv_interfaces` | C++ derlemeli | Özel mesaj ve servis tanımları |
| `usv_core` | Python, ROS'suz | Beyin kütüphanesi: hesap ve karar kodu. ROS'a bağlı olmadığı için testlerle doğrudan denenir, teknede de aynen çalışır |
| `usv_nodes` | Python, ROS düğümleri | Teknede de çalışacak düğümler: `navigator`, `heading_source`, `mapper` |
| `usv_sim` | Python, ROS düğümleri | Simülasyon: Cube ve ZED'in yerine geçen düğümler, parkurlar, tekne modeli, launch |
| `usv_gz_plugins` | C++ | Gazebo eklentisi: gövde şeklinden kaldırma kuvveti |
| `usv_learn` | Python | Öğrenme paketi, sistemin parçası değil |

### 3.2 Dosya adlandırma kuralı (`usv_core`)

- `teknofest_` ile başlayan: yalnızca TEKNOFEST (donmuş).
- `roboboat_` ile başlayan (bir de `world_map.py`): yalnızca RoboBoat.
- Öneki olmayan: ortak.

Dosyalar işe göre bölünür; her TEKNOFEST dosyasının RoboBoat'ta bir eşi yoktur.

### 3.3 `usv_core` dosyaları

| Dosya | Ne yapar | |
|---|---|---|
| `nav_base.py` | İki beynin ortak kısmı: yer istasyonu komut kuyruğu, manuel/otomatik, acil durum kilidi, röle isteği, karıştırıcı | Ortak |
| `commands.py` | Komut süzgeci: başka araca giden komutu eler, tekrar gönderileni atlar, onay (ack) yönetir | Ortak |
| `safety.py` | Acil durum kilidi; kalkış kontrolü (GPS enlemi ≠ 0 ve pruva ≠ 0) | Ortak |
| `relay.py` | Röle komutunu tekrar deneme (teknede `fc_bridge` kullanacak) | Ortak |
| `mixer.py` | "İleri + dön" komutunu dört itici ve dümen PWM'ine çevirir (§4.7) | Ortak |
| `planner.py` | A* yol bulma, yol düzeltme, pure pursuit hesabı (eski koddan, değiştirilmedi) | Ortak |
| `costmap.py` | A*'ın çalıştığı ızgara: tekne merkezli 40 × 40 m, 10 cm hücre | Ortak |
| `navigation.py` | Coğrafi hesaplar: mesafe, yön, bir noktadan şu kadar ötesi | Ortak |
| `heading.py` | ZED pusulası için eski Kalman filtresi, sapma, kaynak seçimi | Ortak |
| `detection.py` | Kutu + derinlik → konum. `process_frame` TEKNOFEST, `measure_boxes` RoboBoat | Ortak |
| `classes.py` | İki yarışmanın sınıf listeleri; RoboBoat sınıflarının yarıçapları | Ortak |
| `nav_config.py` | Navigator ayarları (eski `config.py` değerleri) | Ortak |
| `teknofest_navigator.py` | TEKNOFEST beyni (eski `nav_worker` birebir) | TEKNOFEST |
| `teknofest_mission.py` | TEKNOFEST görev sırası (GPS noktalarıyla) | TEKNOFEST |
| `teknofest_task3.py` | TEKNOFEST Görev 3 kamikaze araması | TEKNOFEST |
| `teknofest_guidance.py` | TEKNOFEST sürüşü (A* + pure pursuit / PID, görev mantığıyla iç içe) | TEKNOFEST |
| `teknofest_costmap.py` | TEKNOFEST yerel çerçevesi, nesne hafızasından ızgara çizimi, Görev 2 koridoru | TEKNOFEST |
| `teknofest_object_memory.py` | Eski kısa ömürlü nesne hafızası | TEKNOFEST |
| `world_map.py` | Kalıcı dünya haritası (§4.5) | RoboBoat |
| `roboboat_course.py` | Haritadan parkuru tanıma (§4.6) | RoboBoat |
| `roboboat_run.py` | Koşu yöneticisi: aşamalar, süreler, olaylar (§4.8) | RoboBoat |
| `roboboat_drive.py` | Yol izleyici ve hareketler: git, kapıdan geç, etrafında dön, ara (§4.9) | RoboBoat |
| `roboboat_tasks.py` | Görev aşamaları; şimdilik Görev 1: giriş, dönüş, çıkış (§4.10) | RoboBoat |
| `roboboat_navigator.py` | RoboBoat beyni: ortak taban + parkur modeli + koşu | RoboBoat |

### 3.4 Diğer paketlerin dosyaları

| Dosya | Ne yapar |
|---|---|
| `usv_nodes/navigator.py` | Navigator düğümü, 25 Hz. `competition` ayarına göre TEKNOFEST ya da RoboBoat beynini çalıştırır |
| `usv_nodes/navigator_io.py` | ROS mesajları ↔ beyin sözlükleri, ayarların okunması |
| `usv_nodes/heading_source.py` | ZED ya da FC pruvasını seçip `/usv/heading` yayınlar (ayar: ZED) |
| `usv_nodes/mapper.py` | Tespitlerden kalıcı haritayı kurar, yayınlar, RViz işaretlerini çizer, kapanışta JSON'a yazar |
| `usv_sim/sim_bridge.py` | Orange Cube'un yerine geçer (§4.2) |
| `usv_sim/fake_perception.py` | ZED ve YOLO'nun yerine geçer (§4.3) |
| `usv_sim/camera.py`, `zed.py`, `sensors.py`, `actuators.py`, `geo.py` | Simülasyon modelleri: kamera, manyetometre, GPS hataları, ESC ve servo, enlem-boylam hesabı |
| `usv_sim/course.py` | Parkur dosyası okuyucu |
| `usv_sim/survey.py`, `survey_drive.py` | Ölçüm rotasını Gazebo'nun gerçek konumuyla süren pilot (yalnızca ölçüm koşusu) |
| `usv_sim/launch/sim.launch.py` | Her şeyi tek komutla açar |
| `usv_sim/scripts/make_course_world.py` | Parkur dosyasından Gazebo dünyası üretir |
| `usv_sim/courses/*.yaml` | Parkurlar: `golcuk_2026_10_14` (TEKNOFEST), `roboboat_2026` |
| `ws/tools/*.py` | Rapor araçları (§5) |

---

## 4. Veri akışı: sensörden motora

### 4.0 Genel şema

```
 GAZEBO (fizik, gerçek)
   │ gerçek konum, yön, hız (/usv/sim/navsat, /usv/sim/imu, /usv/sim/odometry)
   ▼
 sim_bridge ─────────► /usv/fix (GPS, 2 Hz, hatalı)  /usv/speed  /usv/heading/fc
 fake_perception ────► /usv/heading/zed  /usv/imu (jiroskop)  /usv/detections (kamera)
                                │
 heading_source ──► /usv/heading (ZED seçili)
 mapper ──────────► /usv/map (teknenin kendi haritası)
                                │
 navigator (25 Hz)
   parkur modeli ← /usv/map
   koşu aşaması → hareket → yol izleyici → (ileri, dön) → karıştırıcı
   └─► /usv/thrusters/pwm (4 itici + dümen)
                                │
 sim_bridge: PWM → itki (N) ve dümen açısı ──► Gazebo
```

Teknede `sim_bridge`'in yerine `fc_bridge`, `fake_perception`'ın yerine gerçek `perception`
gelecek. Aradaki her şey (`heading_source`, `mapper`, `navigator`) olduğu gibi kalacak.

### 4.1 Gazebo ve tekne modeli

- **Dünya:** VRX'in Nathan Benderson Park dünyası (RoboBoat'un yapıldığı göl). Parkur dosyasından
  her açılışta yeniden üretilir (`/tmp/usv_sim_<parkur>.sdf`).
- **Tekne (`models/usv_boat`):** 1,18 × 0,78 m katamaran, 45 kg. Gövde ağı CAD STL'den.
  Kaldırma kuvveti `HullBuoyancy` eklentisinden. Dört itici: arka ikisi sabit, ön ikisi dümen
  servosuyla ±45° döner. Direnç katsayıları 14 Ekim videosundan ayarlandı (1680 PWM'de
  ~1,3 m/s).
- **Ölçülmüş yerler:** ZED gövde merkezinin 0,29 m önünde, 0,18 m yukarıda. GPS anteni ZED'in
  0,75 m gerisinde, 0,50 m yukarısında (1 Kasım'da şeritle ölçtün).

### 4.2 `sim_bridge`: Orange Cube'un yerine

**Komut tarafı** (navigator'ın PWM'i → Gazebo):
1. Eski kodun yazma seyreltmesi: değer 3 µs'den az değiştiyse ve son yazmadan bu yana 0,5 s
   geçmediyse gönderilmez. Cube son değeri tutar.
2. 0,5 s komut gelmezse bütün çıkışlar 1500'e (nötr) döner.
3. İtici: PWM → itki tablosu, doğrusal: 1100 → −64 N, 1500 → 0, 1900 → +91 N. Röle kapalıysa 0.
   Örnek: 1680 → 41 N.
4. Dümen: PWM → açı: 1100 = 45° sol, 1900 = 45° sağ. Servo açıyı en fazla 5,2 rad/s hızla
   değiştirir (MG995).
5. Röle ve acil durum servisleri: `/usv/set_relay`, `/usv/emergency_stop`.

**Sensör tarafı** (Gazebo'nun gerçeği → teknenin göreceği):

| Topic | Nasıl üretiliyor |
|---|---|
| `/usv/fix` | Gerçek anten konumu + yavaş dolaşan kayma (σ 1 m, ~60 s'de yön değiştirir) + titreme (σ 0,15 m), 0,1 s gecikme, saniyede 2 kez, 1e-7° adım. RTK'sız Here Pro taklidi |
| `/usv/speed` | Gerçek hız + σ 0,05 m/s, 5 Hz |
| `/usv/heading/fc` | Gerçek yön + σ 1°, 5 Hz (Cube pusulası; şu an kullanılmıyor) |

### 4.3 `fake_perception`: ZED ve YOLO'nun yerine

**Pruva:** Parkur dosyasındaki dünya manyetik alanı + gerçek tekne duruşundan bir
manyetometre okuması hesaplanır, σ 1° gürültü eklenir. Ardından eski kodun Kalman filtresi
ve sapma (Sarasota için −6°) uygulanır → `/usv/heading/zed`. Bu filtre dönüşlerde ~0,25 s
geride kalır (ölçüldü).

**Jiroskop:** Gerçek açısal hız + σ 0,2°/s → `/usv/imu`. Harita dönüş hızını buradan alır.

**Kamera** (saniyede 30 kare, ara sıra 15):
1. Parkurdaki her nesne için: kameranın görüş açısında mı (110°), 20 m içinde mi, önündeki
   bir nesne onu %60'tan fazla örtüyor mu?
2. Görünenler 1280 × 720 görüntüye izdüşürülür → kutu.
3. YOLO taklidi: %98 bulma (kutu 25 pikselden kısaysa daha az), kenarlarda ±2 piksel titreme,
   %2 sınıf karıştırma.
4. Derinlik görüntüsü: nesnenin ön yüzünün kamera eksenindeki uzaklığı (gerçek ZED gibi),
   uzaklığın karesiyle büyüyen hata (3 m'de %1, 15 m'de %5), piksel gürültüsü, %10 boş piksel.

Kamera taklidi nesnelerin gerçek yerini bilmek zorunda; kameranın ne göreceğini hesaplamanın
başka yolu yok. Ama tekneye yalnızca kameranın göreceğini, hatalarıyla birlikte veriyor.

**Ölçüm** (`usv_core.detection.measure_boxes`, teknede de aynı kod) → `/usv/detections`:
1. Derinlik kutunun orta sütununda, alttan %15 yukarıda, 5 × 5 piksellik pencerenin ortancası
   (median) olarak okunur. 0,5–15 m dışındaysa geçersiz sayılır.
2. Yön = pruva + o piksel sütununun kamera eksenine göre açısı.
3. **D2:** mesafe = derinlik ÷ cos(açı). ZED derinliği eksen boyunca verir; yandaki nesne
   olduğundan yakın görünür (45°'de %30).
4. **D6:** mesafeye sınıfın yarıçapı eklenir (direk 0,23 m, küçük şamandıra 0,10 m, sarı
   0,18 m...). Kamera ön yüzü ölçer, harita merkezi ister.
5. **D1:** konum GPS anteninden değil kameradan hesaplanır: anten → 0,75 m ileri, 0,06 m sol
   → oradan mesafe kadar, yön boyunca.

D1, D2, D6 ayar dosyasından ayrı ayrı kapatılabilir (§7.2). Eski TEKNOFEST hattı
(`process_frame` → `/usv/buoys`) değiştirilmedi.

### 4.4 `heading_source`

`/usv/heading/zed` ve `/usv/heading/fc`'yi dinler, ayara göre birini seçer (şu an ZED, eski
koddaki gibi) ve saniyede 50 kez `/usv/heading` olarak yayınlar.

### 4.5 `mapper`: teknenin kendi haritası

Sistem açıldığı anda başlar, manuel sürüşte de çalışır. Haritanın orijini ilk GPS konumudur.
Her kameranın her tespiti için (`usv_core/world_map.py`):

1. Konumu yoksa ya da 12 m'den uzaksa alınmaz (uzak ölçüm güvenilmez).
2. Aynı türden, 1,5 m içindeki en yakın ize katılır; yoksa yeni iz açılır. İki gösterge rengi
   tek tür ("indicator") sayılır, rengi son 15 görülmenin çoğunluğuyla belirlenir.
3. Konumlar ağırlıklı ortalanır: ağırlık = 1 / σ², σ = 0,3 + 0,1 × mesafe. Yakından görülen
   uzaktan görülene baskın gelir.
4. **D4:** hızlı dönüşte pusula geride kaldığı için σ'ya mesafe × tan(0,25 s × dönüş hızı)
   eklenir. 20°/s'den hızlı dönerken yeni iz açılmaz.
5. **D5:** başka türden, en az 5 kat fazla görülmüş ve hiç aynı karede görülmemiş bir izin 1 m
   yakınındaki iz onaylanmaz. YOLO'nun yanlış etiketleri böyle bastırılır. Gerçek komşular
   (kırmızının 0,65 m yanındaki siyah) aynı karede ayrı kutu olarak göründükleri için korunur.
6. Bir iz 3 kez görülünce "onaylı" olur.

Çıktılar: `/usv/map` (saniyede 2), RViz işaretleri `/usv/map/markers`, kapanışta
`/ws/bags/map_<parkur>.json` (her koşuda üzerine yazılır).

### 4.6 Parkur modeli (`roboboat_course.py`)

Navigator haritayı yarım saniyede bir okur ve parkur öğelerini **yalnızca haritadan** tanır.
Kitapçıktaki ölçüler tek ön bilgidir:

| Öğe | Nasıl tanınıyor |
|---|---|
| Görev 1 kapıları | Kırmızı ve yeşil direk, 3–14 m arayla. İlk kapı koşunun başladığı yere yakın olan |
| Kapı çerçevesi | Orijin ilk kapının ortası, "ileri" ikinci kapıya doğru. Tekne kapılardan düz geçerken pusulayı örnekler; o yön çerçevenin yönü olur (GPS hatası taşımaz). İpuçları bu çerçevede okunur |
| Görev 2 kanalı | Küçük kırmızı-yeşil çiftlerin (1,2–4 m genişlik) en uzun, art arda dizilmiş zinciri |
| Enkaz alanı | Kanal çıkışından 1 m ötedeki siyahlar ve göstergeler, 30 m içinde |
| Görev 3 | Kanalın parçası olmayan bir çift, 8–40 m ötesinde sarı şamandıra, arada gösterge (rengi dönüş yönü) |
| Marina | Rakam afişleri; her birine en yakın gösterge (renk) ve gemi (dolu mu). Afişler iskelelere kümelenir; kuzeydeki "N". En iyi yer (yeşil, boş, en küçük numara) ancak **altı yerin hepsi** görülünce verilir |
| Görev 6 bölgeleri | Görev 3'ün sarısı ve marinaya en yakın sarı |

Bulunamayanlar "eksik" listesine yazılır; koşu neyi araması gerektiğini buradan bilir.

### 4.7 Navigator: döngü, iki beyin, karıştırıcı

**Düğüm** saniyede 25 kez döner:
1. Bekleyen yer istasyonu komutlarını işler.
2. Acil durum kilitliyse nötr verir, başka hiçbir şey yapmaz.
3. Beyni çalıştırır (`competition` ayarına göre TEKNOFEST ya da RoboBoat).
4. Çıkan komutu `/usv/thrusters/pwm` olarak yayınlar.

**Başlangıç:** Manuel modda başlar (eski koddaki gibi). Kalkış kontrolü (GPS ve pruva var
mı) geçince görev "başlamış" sayılır. Otomatiğe `/usv/set_manual false` ya da launch'ta
`auto:=true` ile geçilir. Manuelde beyin nötr yayınlar.

**Acil durum:** `/usv/navigator/emergency_stop` aynı döngüde nötr verir ve kilitlenir. Çıkış
için önce röle açılır, sonra görev yeniden seçilir (`/usv/set_task`). Yalnızca röle açmak tekneyi
hareket ettirmez.

**Karıştırıcı** (`mixer.py`, eski kodun birebir aynısı). Girdi: ileri PWM (1500 = dur) ve dönüş
PWM'i (+ = sağa):
- Dümen = 1500 + dönüş, 1100–1900 arasında.
- **Yerinde dönüş** (ileri tam 1500): sol iticiler 1500 + dönüş, sağ iticiler 1500 − dönüş.
- **Giderken dönüş:** fark = dönüş × (1 + (1850 − ileri) / 350), yavaşken daha güçlü.
  Arka iticiler tam farkı, ön iticiler yarısını alır.
- **Geri giderken** (ileri < 1450) fark yok.
- Her kanal 1100–1900 arasında tutulur ve döngü başına en fazla 60 µs değişebilir
  (ani sıçrama olmasın).

Örnekler:

| İstek | Arka sol / sağ | Ön sol / sağ | Dümen | Ne olur |
|---|---|---|---|---|
| Düz seyir (1680, 0) | 1680 / 1680 | 1680 / 1680 | 1500 | Her itici 41 N, ~1,3 m/s |
| Hafif sağ (1680, +40) | 1739 / 1621 | 1710 / 1650 | 1540 | Sola göre fazla itki + dümen sağa |
| Yerinde sağa (1500, +200) | 1700 / 1300 | 1700 / 1300 | 1700 | Sol ileri, sağ geri |

### 4.8 RoboBoat koşu yöneticisi (`roboboat_run.py`)

Kitapçığa göre (3.5) bir koşu: Görev 1 kapılarından gir, istediğin görevleri yap, aynı
kapılardan ters yönde çık.

```
WAIT_START → T1_ENTRY → task2 → task3 → task5 → RETURN → T1_EXIT → FINISHED
```

- Görev sırası koşudan önce ayar olarak verilir (`roboboat.task_order`).
- Bir görev 240 s içinde bitmezse atlanır.
- Toplam 900 s dolarsa tekne ne yapıyorsa bırakıp kapılara döner. Çıkış tek başına 300 puan
  (raporuyla), bu yüzden dönüş ve çıkış süre sınırıyla kesilmez.
- Manuelde koşu duraklar, otomatikte kaldığı yerden devam eder.
- `/usv/set_task` ile istenen aşamaya atlanır (deneme için).
- Her aşama değişikliği bir olay listesine yazılır (R4 raporları buradan üretilecek).
- Henüz yazılmamış görevler (2, 3, 5) "atlandı" deyip geçer.

### 4.9 Hareketler ve yol izleyici (`roboboat_drive.py`)

Her hareket işini bir **yol**a (nokta listesi) çevirir; tek bir yol izleyici o yolu sürer.

**Yol izleyici (pure pursuit), her döngüde:**
1. Teknenin yol üzerindeki izdüşümünden **bakış mesafesi** kadar ilerideki noktayı bul
   (düz yolda 4,0 m, A* engel yüzünden yolu kırdıysa 2,0 m).
2. Yön hatası = o noktanın yönü − teknenin pruvası (+ = hedef sağda).
3. Dönüş PWM = 2 × hata + 0,8 × (hatanın değişimi).
4. Hata 60°'yi geçerse **yerinde dönüş** (ileri 1500, dönüş ±200), 35°'nin altına inince bırak.
5. Aksi halde ileri = 1500 + 180 × max(0,25, cos(hata)): hata büyükse yavaşla.

Bakış mesafesi neden 4,0 m: TEKNOFEST ayarı (~1,3 m) Gazebo teknesini düz yolda ~10 s
periyotla ±30–40° salındırdı. Yakın hedef noktası, ataleti olan bir teknede aşılıyor. Ölçtük
(§6.3).

**Hareketler:**

| Hareket | Ürettiği yol | Bitti |
|---|---|---|
| `DriveTo` (git) | Haritadaki işaretler engel ızgarasına çizilir (§4.11); önü açıksa düz çizgi, değilse A* yolu; saniyede 5 kez yeniden planlanır | Hedefe 1,5 m |
| `PassGate` (kapıdan geç) | Kapının 4 m önü → ortası → 2 m ötesi, kapıya dik. Kapı o karede kamerada görünüyorsa direklerin konumu o kareden güncellenir | Kapı çizgisinin 2 m ötesi |
| `Circle` (etrafında dön) | Nesnenin çevresinde çember, saat yönü ya da tersi | 360° süpürülünce |
| `Search` (ara) | Önce yerinde bir tur, sonra genişleyen kare | Aranan bulununca ya da desen bitince |

**Kamera ile düzeltme neden önemli:** Harita GPS ile kurulu ve GPS 1–2 m kayıyor. Ama aynı
karedeki tespit ile teknenin o anki GPS'i aynı kaymayı taşır; tekneye göre konum doğru çıkar.
Testte harita kapıyı 1,5 m yanlış yerde gösterirken tekne gerçek ortadan geçti.

### 4.10 Görev 1 aşamaları (`roboboat_tasks.py`)

| Aşama | Ne yapıyor |
|---|---|
| `T1Entry` | Kapı yoksa arar. İlk kapıdan geçer. İkinci kapı başta haritada değil (16 m uzakta, harita 12 m'ye kadar alır): ilk kapının 8 m ötesine düz gider, ikinci kapı haritaya girince ondan geçer. Aradaki düz kısımda pusulayı örnekler → çerçevenin yönü. Giriş zamanı ve konumunu hafızaya yazar |
| `ReturnToGates` | İkinci kapının 6 m parkur tarafına `DriveTo` (A*) |
| `T1Exit` | Önce ikinci, sonra ilk kapıdan, ters yönde (kırmızı sağda). Çıkış zamanı ve konumunu yazar |
| `Task2` (R3.1–R3.2) | Kanal haritada yoksa ipucunun gösterdiği yere gider ve orada arar (ipucu yoksa kapıların 15 m önünde). Kanal bulununca Görev 1'e yakın uçtaki ilk kapının 4 m önüne gider, kapıları sırayla kamera ile düzelterek geçer (bakış mesafesi 2,5 m); ilerideki kapılar yaklaştıkça haritaya girer. Son kapıdan 9 m sonra kapı yoksa kanal bitmiştir. Geçiş yönü kanalın yönünden; kırmızının hangi tarafta olduğuna bakmaz |

### 4.11 TEKNOFEST beyni (donmuş)

Eski `nav_worker`'ın döngü döngü aynısı. Görev noktaları yer istasyonundan GPS olarak
gelir; görevler Görev 1 → 2 → 3 sırasıyla gider; gördüklerini kısa ömürlü nesne hafızasından
(`/usv/buoys`) alır. Altın testler: eski kod sahte donanım ve sanal saatle çalıştırılır, yeni
kodla her döngüde görev durumu ve beş PWM birebir karşılaştırılır (16 senaryo).

### 4.12 "Harita" kelimesi altı ayrı şey

| Ne | Kim üretiyor | Neye göre | Tekne kullanıyor mu |
|---|---|---|---|
| **Gazebo dünyası** | `make_course_world.py`, parkur dosyasından | Gerçek konumlar | Hayır; fizik ve kamera taklidi için |
| **Teknenin dünya haritası** (`/usv/map`) | `mapper` | Kameranın tespitleri + GPS + pusula | Evet, her şeyin temeli |
| **RViz görüntüsü** | `mapper`'ın işaretleri | Teknenin haritası; mavi çizgi GPS izi (gerçek yol değil) | Hayır, gösterim |
| **Parkur modeli** | Navigator, her 0,5 s | Teknenin haritası | Evet: kapılar, kanal, marina |
| **A* ızgarası** | `DriveTo`, her 0,2 s | Teknenin haritası, tekne çevresi 40 m | Evet: engelden kaçma |
| **Rapor resimleri** | `ws/tools/*_report.py`, koşudan sonra | Halkalar: parkur dosyası (hakem). Dolu noktalar: teknenin haritası. Çizgiler: Gazebo'nun gerçek izi | Hayır, değerlendirme |

**Tekne parkur dosyasını hiç okumaz.** Navigator'a parkur dosyasından giden tek bilgi
`mission` bölümü: görev sırası ve ipuçları. İpuçları şimdilik hiçbir aşama tarafından
kullanılmıyor. Kullanılmaya başlayınca, kıyıdan göz kararını taklit etmek için bilerek
birkaç metre hata eklenecek.

---

## 5. Ölçüm ve rapor araçları (`ws/tools`)

| Araç | Ne için | Nasıl çalışıyor |
|---|---|---|
| `survey_drive` (launch `survey:=true`) | Haritayı ölçmek | Navigator kapalı. Parkur dosyasındaki ~155 m'lik rotayı Gazebo'nun **gerçek** konumuyla sürer; her koşu aynı yoldan gider, ölçtüğümüz şeyler (GPS, pusula) yönü etkilemez |
| `map_report.py` | Harita doğruluğu | Her tespiti geldiği gerçek nesneyle eşleştirir (açı ve derinlikten; GPS ve pusula kullanmadan). Girdileri tek tek gerçeğe çekerek hatanın kaynağını ayırır. Üç harita: 3a koşudaki, 3b kayıttan yeniden kurulan, 3c GPS kusursuzmuş gibi kurulan (ölçüm zinciri hedefi). `--map AYAR=DEĞER` ile harita ayarı yeni koşu yapmadan denenir |
| `course_report.py` | Parkur modeli doğruluğu | Parkur modelini bir kez gerçek konumlarla, bir kez haritayla çalıştırır, öğe öğe karşılaştırır; ipuçlarının haritada doğru yeri gösterip göstermediğine bakar |
| `roboboat_report.py` | RoboBoat koşusu | Gazebo'nun gerçek izinden: aşama süreleri, kapı geçişleri (yön, ortadan sapma, gövdenin tamamı direklerin arasından mı), temaslar, dümen ölçüleri. Resim: gerçek iz, GPS noktaları, teknenin haritası, pruva ve dümen grafikleri |
| `run_report.py` | Gölcük (TEKNOFEST) koşusu | Görev zaman çizelgesi, şamandıralara boşluk, videoyla karşılaştırma |

`seed:=N` ile gürültü her koşuda aynı olur; bir değişikliğin öncesi ve sonrası aynı şartlarda
karşılaştırılır.

---

## 6. Ne yaptık, ne bulduk (sırayla)

### 6.1 Faz 4–5 (TEKNOFEST, kısaca)

- Simülasyon köprüleri ve tekne modeli kuruldu; direnç 14 Ekim videosundan ayarlandı.
- Bulgu (Faz 4): eski kod ZED derinliğini mesafe sanıyor ve nesneleri antenden konumluyor.
  Hafızada ortanca 2,2 m hata; ikisi düzeltilince 0,26 m. Eşdeğerlik bozulmasın diye TEKNOFEST
  hattında bilerek korundu.
- Navigator taşındı; 16 senaryoda eski kodla döngü döngü aynı.
- Gölcük koşusu (5.6): Görev 1'den bitişe temassız; ama 126 s (videoda ~68 s). Sebep: tekne
  gördüğü her şamandıranın etrafından dolaşıyor. Pusula +8° hatası tek başına bir temas
  getirdi.

### 6.2 R1: RoboBoat parkuru, algılama, harita

| Adım | Ne | Sonuç |
|---|---|---|
| R1.1–R1.3 | 13 RoboBoat sınıfı (İngilizce), ortak parkur okuyucu, gemiler ve iskeleler, parkur Nathan Benderson Park'ta (yerini sen seçtin) | Parkur Gazebo'da |
| R1.4–R1.5 | Kare başına tespitler, kalıcı harita, RViz | İlk harita ~2 m hatalı |
| R1.6 | Ölçüm rotası + `map_report` | İlk ölçüm: tespit başına 2,75 m. Paylar: derinlik geometrisi 1,74 m, GPS 1,01 m, anten-kamera 0,94 m, pusula 0,34 m |
| R1.7 D1 + D2 | Kameradan konumla, derinlik → mesafe | Ölçüm zinciri haritası 1,74 → 0,28 m (hedef geçti) |
| Anten ölçümü | Şeritle: 0,75 m geride, 0,50 m yukarıda | Model ve ayarlar güncellendi |
| R1.7 D4 + D5 | Dönüşte az güven, sınıf karışması kuralı | Onaylı kopya 20 → 7, hayalet 9 → 5 |
| R1.7 D6 | Sınıf yarıçapı | Ölçüm zinciri haritası **ort. 0,14 m, en fazla 0,30 m** |
| D3 | GPS yaşı | Ertelendi (0,14 m; gerçek GPS hızı bilinmiyor) |

**Hedef ikiye ayrıldı** (senin kararın): ölçüm zinciri (GPS gerçeğe çekilmiş) ortalama < 0,4 m,
en fazla 0,8 m; mutlak harita hatası raporlanır ama hedefi yok, çünkü onu RTK'sız GPS'in
kendi kayması (~1 m) belirliyor.

### 6.3 R2: navigator'ın RoboBoat beyni

| Adım | Ne | Sonuç |
|---|---|---|
| R2.1 | Ortak taban (`nav_base`), koşu yöneticisi, `competition` anahtarı | Altın testler yeşil: TEKNOFEST değişmedi |
| Yeniden adlandırma | TEKNOFEST dosyalarına `teknofest_` öneki (senin isteğin) | Altın testler yeşil |
| R2.2 | Parkur modeli + `course_report` | GPS kusursuzken kapılar ve kanal 0,11–0,14 m; teknenin haritasında 0,8–1,5 m. Marina mantık hatası bulundu ve düzeltildi (en iyi yer artık tamamı görülmeden verilmiyor) |
| R2.3 | Hareketler; `guidance` ve `costmap` ayrıldı | Testte kapı ortasından 0,39 m; harita 1,5 m yanlışken kamera ile tam orta |
| R2.4 | Görev 1 aşamaları + `roboboat_report` | İlk Gazebo koşusu temiz ama zikzak (giriş 26 s) |
| Zikzak teşhisi | Rapora dümen ölçüleri ve resim | Pusula gecikmesi 3–6°, GPS değil; sebep kısa bakış mesafesi |
| Bakış mesafesi | 1,3 / 2,5 / 4,0 m ölçüldü | Koşu 54,6 / 46,0 / **43,5 s**; 4,0 seçildi. A* kırık yolda köşe kestiği için orada 2,0 m |
| **R2 bitti** | 3 farklı gürültü (seed 1, 2, 3) | Giriş ve çıkış temiz, temas yok, koşu 43,5 / 45,5 / 46,5 s |

---

## 7. Komut sözlüğü

### 7.1 Simülasyonu açmak

```bash
ros2 launch usv_sim sim.launch.py course:=<parkur> <seçenekler>
```
Sırasıyla: parkur dosyasından Gazebo dünyasını üretir; Gazebo, Gazebo–ROS köprüsü,
`sim_bridge`, `fake_perception`, `heading_source`, `mapper` açılır; seçeneğe göre navigator,
RViz, ölçüm pilotu, kayıt. Ctrl+C hepsini kapatır (harita JSON'u o anda yazılır).

### 7.2 Launch seçenekleri

| Seçenek | Ne yapar |
|---|---|
| `course:=roboboat_2026` | Parkur (`golcuk_2026_10_14` varsayılan) |
| `gui:=false` | Gazebo penceresiz (daha hafif) |
| `paused:=true` | Duraklatılmış başlar |
| `rviz:=true` | RViz'de teknenin haritası, canlı |
| `navigator:=false` | Navigator açılmaz (elle sürmek için) |
| `auto:=true` | Navigator otomatik modda başlar |
| `start_task:=...` | TEKNOFEST başlangıç görevi |
| `survey:=true` | Ölçüm rotası (navigator kapalı) |
| `seed:=N` | Gürültü her koşuda aynı |
| `record:=<ad>` | `/ws/bags/<ad>` klasörüne kayıt |
| `compass_offset_deg:=8.0`, `detect_prob:=0.5`, `max_range_m:=10.0` | Deney: pusula hatası, YOLO kaçırması, menzil |
| `true_distance:=false`, `from_camera:=false`, `add_radius:=false` | D2, D1, D6'yı kapat (eski hal) |
| `lookahead_m:=4.0` | RoboBoat yol izleyicisinin bakış mesafesi |
| `channel_lookahead_m:=2.5` | Görev 2 kanalında bakış mesafesi |
| `hint_error_m:=3` | İpuçlarını rastgele yönde kaydır (kıyıdan tahmin taklidi) |
| `course_east:= course_north:= course_heading:=` | Parkuru başka yere koy |

### 7.3 Sık kullandığımız komutlar

| Komut | Ne yapar, neden kullandık |
|---|---|
| `ros2 launch usv_sim sim.launch.py auto:=true` | Gölcük'ü otomatik başlat (TEKNOFEST) |
| `ros2 service call /usv/set_manual std_srvs/srv/SetBool "{data: false}"` | Manuelden otomatiğe al (yer istasyonunun "otonom" komutu) |
| `ros2 service call /usv/set_task usv_interfaces/srv/SetTask "{task: T1_ENTRY}"` | Görev/aşama seç; acil durumdan çıkışta da gerekli |
| `ros2 service call /usv/navigator/emergency_stop std_srvs/srv/Trigger` | Acil durum |
| `ros2 topic echo /usv/nav_status --once` | Navigator'ın durumu (aşama, mod) |
| `ros2 topic hz /usv/detections` | Bir topic'in gerçek yayın hızı |
| `ros2 topic info /usv/thrusters/pwm` | Kaç yayıncı var (iki yayıncı = çakışma) |
| `ros2 topic pub -r 20 /usv/thrusters/pwm ...` | Elle sürmek (navigator kapalıyken) |
| `ros2 launch ... course:=roboboat_2026 survey:=true seed:=1 record:=survey_x` | Harita ölçüm koşusu |
| `python3 /ws/tools/map_report.py /ws/bags/survey_x --plot ...` | Harita doğruluğu |
| `python3 /ws/tools/course_report.py /ws/bags/<ad> [--gps-true]` | Parkur modeli doğruluğu |
| `ros2 launch ... course:=roboboat_2026 auto:=true record:=t1_x` | RoboBoat koşusu (Görev 1) |
| `python3 /ws/tools/roboboat_report.py /ws/bags/t1_x --plot ...` | Koşunun gerçek izden değerlendirmesi |
| `ros2 bag info /ws/bags/<ad>` | Kayıtta ne var |
| `ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id usv_map` + `rviz2 -d .../map.rviz` + `ros2 bag play ... --topics /usv/map/markers` | Koşuyu RViz'de tekrar izlemek |
| `cp /ws/bags/map_roboboat_2026.json ...` | Harita JSON'unu saklamak (her koşuda üzerine yazılır) |

---

## 8. Bilinen sınırlar ve açık konular

| Konu | Durum |
|---|---|
| **Haritada kopya izler** | GPS kayması giriş ile çıkış arasında 1,5 m'yi aşınca aynı direk iki kez haritalanıyor. Görev 2 raporundan önce, yeniden görülen işaretlerden GPS kaymasını tahmin ederek çözülecek |
| **Mutlak harita hatası** | ~1–1,6 m, RTK'sız GPS'ten. Gerçek GPS'in kaymasını ölçmek gerek (open-items 4f) |
| **İpuçları simülasyonda kusursuz** | Kullanılmaya başlayınca bilerek hata eklenecek |
| **Pusula gecikmesi** | Hızlı dönüşte ~5–9°. Kalıcı çözüm pusula filtresini jiroskopla desteklemek; navigator'ı da etkilediği için ayrı adım |
| **Kanalda bakış mesafesi** | 4,0 m dar kapılarda (2,5 m) köşe kestirebilir; R3'te ölçülecek |
| **Simülasyondaki varsayımlar** | Dümen açısı ±45°, yanal direnç, GPS kayması, YOLO hata oranları ölçülmedi. Gerçek veriler gelince değişecek |
| **Teknede ölçülecekler** | Dümen açısı, ZED pusula doğruluğu, ArduPilot `.param`, GPS kayması (open-items) |

---

## 9. Bu inceleme sırasında belgelerde düzeltilenler

- `ARCHITECTURE.md` §4 node şeması: `mapper`, `heading_source`, RoboBoat topic'leri ve
  `competition` anahtarı eklendi; §10'da `fake_perception`'ın RoboBoat çıktıları eklendi.
- `daily-startup.md`: `usv_nodes` test sayısı 8 → 12.
- Geri kalanı (günlük rehberin RoboBoat bölümleri, launch seçenekleri, sorun giderme,
  open-items'taki R1–R3 satırları, ARCHITECTURE §15–16) bu adımlarda zaten güncellenmişti.
