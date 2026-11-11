# Claude için notlar (usv_ros2)

Barış'ın insansız deniz aracı (İDA) yazılımı: eski kod ROS 2 Jazzy'ye taşınıyor. Simülasyon
Docker içinde (Gazebo Harmonic + VRX), gerçek tekneler Jetson. Hedef: RoboBoat 2027
(18 Şubat 2027), iki tekne: **Azure** (AGX Thor) ve **Marine** (AGX Orin, atış sistemi).

## Önce oku
- `docs/roboboat-2027.md`: güncel iş; §6 tekneler ve haberleşme, §8 yol haritası (Adım 1–8)
- `docs/daily-startup.md`: günlük komutlar (ana makine / container), test ve kayıt
- `docs/ARCHITECTURE.md`: mimari, konular, servisler, kararlar
- `docs/open-items.md`: bekleyenler
- `docs/sistem-raporu.md`: kodun haritası, veri akışı

## Durum (10 Kasım 2026)
- 2026 görev çalışması durdu (Task 1–2 simülasyonda çalışıyor, Proof of Readiness videosu
  için duruyor). TEKNOFEST kodu donmuş; altın testler koruyor, dokunma.
- Adım 1 (isim alanları) bitti: commit `bb16137`. Düğümler `/azure/...` altında; Gazebo koşusu
  temiz (FINISHED 245 s, temas yok).
- Adım 2 (tekne profilleri) bitti: commit `4cf5496`. `ws/src/usv_nodes/config/boats/`,
  `usv_core/boat.py`.
- Adım 3 bitti: commit `bce266c`. Parkur `roboboat_2027` (2026 Task 1–2),
  `start:=readiness|competition`, `mission:=readiness`.
- Adım 4a bitti: commit `8bffdca` (RoboBuoy, ışıklar, rastgele Task 1 alanı).
- Adım 4b bitti: commit `338b325` (marina, pencere ışıkları, Primer yerleşimi).
- Adım 4e bitti: commit `a7575b1` (Task 4 şekilli tekneler). Pinger simüle edilmedi.
- Adım 5a bitti: commit `2195062` (iki hatlı GCS bağlantısı, `telemetry_bridge`).
- Adım 5b bitti: GCS depoda (`gcs/gcs.py`, ROS'suz, PySide6; iki tekne paneli, Ubiquiti +
  RFD). GCS'yi Claude çalıştıramaz (PySide6 yalnız Barış'ın `~/gcs-venv`'inde).
- Konum düzeltmesi (11 Kasım): VRX park modeli gerçek parka göre 76 m kuzeyde, 35 m batıdaydı
  (Barış GCS haritasında yarımadanın GB köşesinden ölçtü). Parkur dosyalarındaki `origin`
  27.366566, -82.450768 oldu. Tek noktadan ölçüldü; Barış'ın doğrulaması ve commit'i
  bekleniyor. Eski kayıtlar eski origin'le (27.367256, -82.451120).
- Adım 5c bitti: commit `f3de9fe` (`usv_interfaces/TaskRequest`, GCS komutu `task`).
- Adım 5d bitti (`5382a12`): `gcs/robocommand/` (RoboBoat 2026 resmi protobuf, TCP, heartbeat/hız
  köprüsü, kural denetleyen sahte sunucu, GCS kutusu). Barış'ın denemesi ve commit'i
  bekleniyor. MQTT taşıması 2027 el kitabı isterse. Veri yolu: tekne → GCS → Ethernet →
  RoboCommand.
- Adım 6a bitti: `usv_core/mission.py` (görev yöneticisi). Readiness eski `RoboBoatRun`'da
  donduruldu (yalnız video).
- Adım 6b yazıldı: navigatör `mission:=roboboat_2027` ile görev yöneticisini kullanır,
  `task_request`'i dinler, sıradakiler GCS'de. Barış'ın testi ve commit'i bekleniyor.
  Sıradaki 6c (`rally`, `hold`, eve dönüş).
- Ana makinede (Claude) çekirdek testleri: `~/gcs-venv/bin/python` (numpy, pytest var; cv2
  yok, gerekirse sahte modülle).
- GCS'yi Claude ana makinede `QT_QPA_PLATFORM=offscreen ~/gcs-venv/bin/python` ile pencere
  açmadan deneyebilir; GCS testleri `~/gcs-venv/bin/python -m pytest gcs/test`.
- Sıra (Barış'la, 11 Kasım): 5d (RoboCommand) → Adım 6 (görev yöneticisi) → Task 1 davranışı
  → marina Core → Task 5 → Task 2 Core; Adım 7-8 donanım hazır oldukça; Task 4 atıştan,
  Task 2 pinger seviyeleri hidrofondan sonra (docs/roboboat-2027.md §8 "Sıra").
- Bilinen: raporda "yeşil 330°" uyarısı GPS/harita kaymasından (R3.5 ertelendi).

## Çalışma şekli (değişmez kurallar)
- Barış'tan beklenenler `docs/baris-yapacaklar.md`'de: veri seti adları, ölçümler, donanım
  soruları, takvim. Yeni bir varsayım ya da Barış'a düşen iş çıkınca oraya ekle. Barış
  "benden beklenen ne var?" diye sorunca o listeyi özetle.
- Türkçe konuş. Kod, yorumlar, commit mesajları ve ROS adları İngilizce.
- **Container'da hiçbir komutu sen çalıştırma.** Barış öğrenmek için kendisi çalıştırır:
  komutları tek tek, kopyalanabilir, nerede çalışacağı (ana makine / container) ve
  "Beklenen:" çıktısıyla ver. Kurulum da yapma.
- `~/usv_ros2` içindeki dosyaları okuyup düzenleyebilirsin. Ana makinede yalnız okuma ve
  saf Python kontrolleri (derleme, `ws/tools/dev/replay.py`) serbest.
- Commit ve push'u Barış yapar; iş bitince `git add` / `git commit` / `git push`
  komutlarını ver. Commit mesajı İngilizce, sonunda `Co-Authored-By` satırı.
- Önce analiz, sonra kod. Tahmin etme, ölç (kayıt + rapor + yeniden oynatma). Kapsamı Barış
  belirler: yol haritasında sıradaki adıma o "geçelim" deyince geç.
- Bir adım bitince ilgili `docs/` dosyasını ve buradaki "Durum" bölümünü güncelle.

## Kısa teknik notlar
- Kod `ws/src`: `usv_core` (saf mantık, ROS'suz, testli), `usv_nodes` (düğümler),
  `usv_sim` (Gazebo, sahte algılama, parkurlar), `usv_interfaces` (mesajlar).
- Konu adları göreli; isim alanı teknenin adı (`sim.launch.py boat:=azure|marine`).
  Parametre dosyalarının anahtarı `/**`.
- Test (container): `cd /ws/src/<paket> && python3 -m pytest test/` (flake8 ve pep257
  dahil; satır en çok 99 karakter).
- Kayıtlar `ws/bags/`; rapor `ws/tools/roboboat_report.py`, yeniden oynatma
  `ws/tools/dev/replay.py` (ROS gerekmez).
