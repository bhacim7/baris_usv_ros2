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
- Adım 4e yazıldı: Task 4 tekneleri yelken levhasında şekille (`circle`, `square` sınıfları).
  Barış'ın testi ve commit'i bekleniyor. Pinger simüle edilmedi.
- Sıradaki: Adım 5, RoboCommand ve haberleşme (GCS görselleri Barış'tan istenecek).
- Bilinen: raporda "yeşil 330°" uyarısı GPS/harita kaymasından (R3.5 ertelendi).

## Çalışma şekli (değişmez kurallar)
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
