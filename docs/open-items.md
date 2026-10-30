# Bekleyenler

Bir şey tamamlandıkça buradan sil ya da üstünü çiz. Son güncelleme: 22 Ekim 2026.

---

## 1. Senden beklenen dosya ve bilgiler

| # | Ne | Neden | Nereye | Ne zaman |
|---|---|---|---|---|
| 1 | Eski sistemin bir koşusundan **telemetri CSV'si** (`arac_telemetri_YYYYMMDD_HHMMSS.csv`). En iyisi Parkur 1'de uzun düz gidişi olan bir koşu | İleri yön su direnci (`xU`, `xUU`) 14 Ekim videosundan ayarlandı (1680 PWM'de ≈1,28 m/s). CSV ile iki şey doğrulanacak: (a) bu hız, başka yönlerde de (rüzgâr etkisi); (b) HUD'daki HIZ değerinin (VFR_HUD groundspeed) GPS konumuna göre neden yarı yarıya düşük okuduğu | `~/usv_ros2/docs/data/`. Git'e eklenip eklenmeyeceğine sen karar ver | Bulabildiğinde; işi bekletmiyor |
| 2 | ~~**STL dosyası**~~ **Tamam (26 Ekim):** `ROBO27_MAIN_BODY_SIM.STL`'den (yapım öncesi hâl) yalnızca iki gövde alındı. Açık soru: STL'de gövde merkezleri arası 0,60 m, çizimde 0,52 m; modelde 0,52 kullanılıyor. Gerçek teknede ölçüp söyle | | | |
| 3 | ~~İtici bataryası~~ **Cevaplandı:** her iticiye bir 7S. Modelde 91 N (8 kgf @24 V, 7S'e ölçeklendi). Not: üretici Utras'ı 3S–6S için veriyor | | | |
| 4 | **Dümen servosunun** gerçek açı aralığı (servo: 2× MG995, hız datasheet'ten) | Şu an ±45° (1100 = 45° sol, 1900 = 45° sağ) kabul edildi | Bana yaz | Teknede ölçebildiğinde |
| 4c | **ArduPilot parametre dosyası** (Mission Planner → Config → Full Parameter List → Save, `.param`) | GPS hızı, akış hızları, servo kanalları, röle başlangıç durumu, pusula ayarları; `sim_bridge` değerleri şimdilik koddan ve varsayımdan | Bana gönder | Bulabildiğinde |
| 4b | ~~**Video** (14 Ekim, eski kodla hareket)~~ **Tamam:** 2123 karenin hepsi okundu (23 Ekim). İleri hız modele işlendi, dönüş hızı modelle uyumlu | | | |
| 1b | **14 Ekim koşusunun dosyaları** (bulunamadı, silinmiş olabilir): `final_costmap.npz`, `kayit_20261014_*.mp4` (tam çözünürlük), `arac_telemetri_20261014_*.csv`. Başka koşularınki de olur | Gölcük parkuru şu an videodan TAHMİNİ (±2 m). `.npz` gerçek şamandıra konumlarını, CSV pusula hatasını verir | `~/usv_ros2/ws/bags/legacy_runs/` | Bulunursa |
| 5 | Elinizde varsa ZED'in ham kayıtları (**`.svo`**) | `perception` node'unu tekneye çıkmadan, gerçek görüntü ve derinlikle geliştirmek için | Bana sor, yeri birlikte belirleriz | Faz 6'dan önce, isteğe bağlı |
| 6 | YOLO modelinin **`.pt`** dosyası | Thor'da `.engine`'e yeniden dönüştürülecek (eski `.engine` yeni donanımda çalışmaz). Depoya konmayacak | `~/usv_models/` (depo dışında) | Faz 6 |
| 7 | **RoboBoat kural belgesi** | Uyarlama aşamasında görevler ve raporlama buna göre tasarlanacak | Bana gönder | Faz 8 |
| 8 | Parkur 3'te ceza alan noktaların hangileri olduğu | Arama mantığının girdileri | Sahada güncellenecek | Saha testleri |

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

Faz 4 (sim_bridge, fake_perception, sim launch) bitti. Faz 5 (navigator taşıma):

1. ~~5.1 altın referans düzeneği~~, ~~5.2 Görev 1-2 navigator~~ (commit a0cb421), ~~5.3 Görev 3 araması~~.
2. 5.4 acil durum kilidi, kalkış kontrolü, röle tekrarları (yazıldı, test bekliyor). Karar (31 Ekim): acil durum röleyi kesmez.
3. 5.5 ROS `navigator` node'u, servisler, NavStatus, `/usv/plan`, launch'a ekleme.
4. 5.6 Gazebo'da tam Gölcük koşusu ve videoyla karşılaştırma.

Mimari kararları ve tam yol haritası: `ARCHITECTURE.md`.

---

## 4. Altın testlerle kapsanmayan kısımlar

Bu satırlar eski kodla döngü döngü karşılaştırılmadı; bir taşıma hatası olursa ancak o yol çalışınca görülür.

| Kısım | Neden kapsanmadı | Ne zaman bakılır |
|---|---|---|
| Görev 3 çapa + devriye (`task3.py`: GOTO_STANDOFF, PATROL_1/2) | TEKNOFEST'e özgü; RoboBoat'ta değişecek (31 Ekim kararı) | TEKNOFEST'e tekrar gidilirse `task3_anchor` senaryosu yazılır |
| Röle tekrar denemeleri | Eski kodla karşılaştırmalı birim testi var (`test_relay.py`); gerçek uçuş kontrolcüsüyle değil | Faz 6, `fc_bridge` iskelede |
