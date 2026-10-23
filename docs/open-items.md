# Bekleyenler

Bir şey tamamlandıkça buradan sil ya da üstünü çiz. Son güncelleme: 22 Ekim 2026.

---

## 1. Senden beklenen dosya ve bilgiler

| # | Ne | Neden | Nereye | Ne zaman |
|---|---|---|---|---|
| 1 | Eski sistemin bir koşusundan **telemetri CSV'si** (`arac_telemetri_YYYYMMDD_HHMMSS.csv`). En iyisi Parkur 1'de uzun düz gidişi olan bir koşu | Simülasyondaki ileri yön su direnci (`xU`, `xUU`) şu an tahmin. CSV'deki GPS hızı (`yer_hizi_mps`) ve PWM (`hiz_set_pwm`) ile gerçek değeri hesaplanacak | `~/usv_ros2/docs/data/`. Git'e eklenip eklenmeyeceğine sen karar ver | Bulabildiğinde; işi bekletmiyor |
| 2 | Teknenin SolidWorks montajından **STL dosyası** (ayarlar aşağıda) | Simülasyondaki tekne şu an kutulardan oluşuyor; görünüş gerçek tekneye benzesin. Fiziği etkilemez | `~/usv_ros2/ws/src/usv_sim/models/usv_boat/meshes/usv_boat.stl` | Bulabildiğinde |
| 3 | ~~İtici bataryası~~ **Cevaplandı:** her iticiye bir 7S. Modelde 91 N (8 kgf @24 V, 7S'e ölçeklendi). Not: üretici Utras'ı 3S–6S için veriyor | | | |
| 4 | **Dümen servosunun** gerçek açı aralığı | Şu an ±45° (1100 = 45° sol, 1900 = 45° sağ) kabul edildi | Bana yaz | Teknede ölçebildiğinde |
| 4b | **Video** (14 Ekim, eski kodla hareket) | Kare kare inceleme: dönme ve hız davranışı | `~/usv_ros2/ws/bags/video/` (git dışı) | Şimdi |
| 5 | Elinizde varsa ZED'in ham kayıtları (**`.svo`**) | `perception` node'unu tekneye çıkmadan, gerçek görüntü ve derinlikle geliştirmek için | Bana sor, yeri birlikte belirleriz | Faz 6'dan önce, isteğe bağlı |
| 6 | YOLO modelinin **`.pt`** dosyası | Thor'da `.engine`'e yeniden dönüştürülecek (eski `.engine` yeni donanımda çalışmaz). Depoya konmayacak | `~/usv_models/` (depo dışında) | Faz 6 |
| 7 | **RoboBoat kural belgesi** | Uyarlama aşamasında görevler ve raporlama buna göre tasarlanacak | Bana gönder | Faz 8 |
| 8 | Parkur 3'te ceza alan noktaların hangileri olduğu | Arama mantığının girdileri | Sahada güncellenecek | Saha testleri |

### STL dışa aktarma (SolidWorks)

1. Montajı aç → **File → Save As** → dosya türü **STL (\*.stl)**.
2. **Options**:
   - Output as: **Binary**
   - Unit: **Meters** (milimetrede bırakılırsa söyle, ölçeği ayarlarım)
   - Resolution: **Coarse**
   - **Do not translate STL output data to positive space**: seçili (orijin korunur)
   - Montaj **tek dosya** olarak kaydedilsin
3. Dosya 20 MB'tan büyükse çözünürlüğü düşür.

---

## 2. Senin yapacağın kontroller

| # | Kontrol | Nasıl | Beklenen |
|---|---|---|---|
| 1 | Simülasyon teknesinin su çekimi | `daily-startup.md` → "Kendi simülasyonumuz" → konum döngüsü | z sıfır çevresinde, birkaç cm oynuyor |
| 2 | Yerinde dönüş hızı | Sol iticiler +10 N, sağ iticiler -10 N; bir turun süresini say | Sağa dönüş, bir tur ≈ 15 s (≈ 24°/s) |
| 3 | Dümenle dönüş yönü | Dördü +10 N, ön iticiler +0,5 rad, sonra -0,5 rad | +0,5'te sola, -0,5'te sağa; yana kayma az |
| 4 | Simülasyon teknesindeki değişiklikleri kaydet | Kontroller tamamsa: `cd ~/usv_ros2 && git add . && git commit -m "Four-thruster catamaran model with revised buoyancy and drag" && git push` | GitHub'a gider |
| 5 | Laptop donması | PSR 13 Ekim'de kapatıldı; donma tekrar ederse `daily-startup.md` → "Ekran donarsa" | Hiç donmaması |
| 6 | Pil şarj sınırı | Şarj aleti takılıyken `cat /sys/class/power_supply/BAT0/status /sys/class/power_supply/BAT0/capacity` | %80'in üstünde `Not charging` |

---

## 3. Sırada (Claude ile)

1. `sim_bridge`: Gazebo topic'lerini ROS 2 topic'lerine bağlayan node. `/usv/thrusters/pwm` → dört itici ve ön itici açısı; teknenin GPS, IMU ve pruva bilgisi → `/usv/fix`, `/usv/heading`, `/usv/speed`. Tekneye GPS ve IMU sensörleri de bu adımda eklenecek.
2. `fake_perception`: dünyaya şamandıralar koyup `/usv/buoys` yayınlamak.
3. Faz 5: `navigator`'ın bölme planı (koddan önce sunulacak).

Mimari kararları ve tam yol haritası: `ARCHITECTURE.md`.
