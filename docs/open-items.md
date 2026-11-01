# Bekleyenler

Bir şey tamamlandıkça buradan sil ya da üstünü çiz. Son güncelleme: 31 Ekim 2026 (Faz 6, R1.6 sonrası).

---

## 1. Senden beklenen dosya ve bilgiler

| # | Ne | Neden | Nereye | Ne zaman |
|---|---|---|---|---|
| 1 | Eski sistemin bir koşusundan **telemetri CSV'si** (`arac_telemetri_YYYYMMDD_HHMMSS.csv`). En iyisi uzun düz gidişi olan bir koşu | İleri yön su direnci 14 Ekim videosundan ayarlandı (1680 PWM'de ≈1,28 m/s). CSV ile iki şey doğrulanacak: (a) bu hız başka yönlerde de geçerli mi (rüzgâr); (b) HUD'daki HIZ değeri (VFR_HUD) neden GPS'e göre yarı yarıya düşük | `~/usv_ros2/docs/data/` | Bulabildiğinde; işi bekletmiyor |
| 1b | **14 Ekim koşusunun dosyaları** (bulunamadı, silinmiş olabilir): `final_costmap.npz`, `kayit_20261014_*.mp4` (tam çözünürlük), `arac_telemetri_20261014_*.csv`. Başka koşularınki de olur | Gölcük parkuru videodan TAHMİNİ (±2 m). `.npz` gerçek şamandıra konumlarını, CSV pusula hatasını verir | `~/usv_ros2/ws/bags/legacy_runs/` | Bulunursa |
| 2 | ~~STL dosyası~~ **Tamam (26 Ekim).** Açık soru: STL'de gövde merkezleri arası 0,60 m, çizimde 0,52 m; modelde 0,52 kullanılıyor. Gerçek teknede ölçüp söyle | | | |
| 3 | ~~İtici bataryası~~ **Cevaplandı:** her iticiye bir 7S | | | |
| 4 | **Dümen servosunun** gerçek açı aralığı (2× MG995) | Şu an ±45° kabul edildi. Simülasyon teknesi virajlarda videodakinden çok yavaşlıyor; düz giderken hız doğru (1,33 m/s). En olası sebep bu açı (ve tahmini yanal sürtünme) | Bana yaz | Teknede ölçebildiğinde |
| 4b | ~~Video (14 Ekim)~~ **Tamam (23 Ekim)** | | | |
| 4c | **ArduPilot parametre dosyası** (Mission Planner → Config → Full Parameter List → Save, `.param`) | GPS hızı (`GPS_RATE_MS`; şu an videodan 2 Hz kabul ediliyor), akış hızları, servo kanalları, röle başlangıç durumu, pusula ayarları | Bana gönder | Bulabildiğinde |
| 4d | **ZED pusulasının teknede doğruluğu**: tekneyi bilinen yönlere (ör. iskele boyunca, 4 yön) çevirip HUD'daki yönü telefon pusulası / harita ile karşılaştır | Şamandıralar pusulayla konumlanıyor: 10 m'de 8° ≈ 1,4 m yana kayma. Faz 5.6'da 8° hata tek başına bir temas getirdi | Değerleri bana yaz | İlk iskele testi |
| 4e | ~~GPS anteni ile ZED arası mesafe~~ **Tamam (1 Kasım):** anten ZED'in 0,75 m gerisinde, 0,50 m üstünde. Model ve ayarlar güncellendi. Açık küçük soru: ikisi de teknenin orta çizgisinde mi? (öyle kabul edildi) | | | |
| 4f | **GPS'in durduğu yerde kayması**: tekne iskelede dururken 10 dakika `/usv/fix` kaydı (ya da Mission Planner'dan konum logu) | Haritanın mutlak hatasını GPS alıcısı belirliyor. Simülasyon 1 m / 60 s kayma VARSAYIYOR; gerçek değer harita hedefini doğrudan etkiler | Kayıt dosyası | Faz 7, iskele |
| 5 | Elinizde varsa ZED'in ham kayıtları (**`.svo`**) | `perception` node'unu tekneye çıkmadan, gerçek görüntü ve derinlikle geliştirmek için | Bana sor | Faz 7'den önce, isteğe bağlı |
| 6 | YOLO modelinin **`.pt`** dosyası, RoboBoat sınıflarıyla | Thor'da `.engine`'e dönüştürülecek (eski `.engine` yeni donanımda çalışmaz). Depoya konmayacak | `~/usv_models/` (depo dışında) | Faz 7 |
| 6b | **Veri setindeki sınıf adlarını İngilizceye çevirmek**: `buoy_black`, `buoy_green`, `buoy_red`, `buoy_yellow`, `digit_1`, `digit_2`, `digit_3`, `indicator_green`, `indicator_red`, `plus`, `pole_green`, `pole_red`, `triangle` (alfabetik; kod bu sırayı bekliyor, `usv_core/classes.py`) | Kod ile modelin sınıf numaraları aynı olmalı | Veri setinde | Model yeniden eğitilmeden önce |
| 7 | ~~RoboBoat kural belgesi~~ **Tamam:** 2026 Team Handbook alındı. **2027 handbook** çıkınca (Kasım 2026 civarı) gönder; farklar uyarlanacak | | | |

---

## 2. Senin yapacağın kontroller

| # | Kontrol | Nasıl | Beklenen |
|---|---|---|---|
| 1–4 | ~~Simülasyon teknesinin su çekimi, dönüş hızı, dümen yönü, model commit'i~~ **Tamam** (Faz 4, commit `a2438ed`) | | |
| 5 | Laptop donması | PSR 13 Ekim'de kapatıldı; donma tekrar ederse `daily-startup.md` → "Ekran donarsa" | Hiç donmaması |
| 6 | Pil şarj sınırı | Şarj aleti takılıyken `cat /sys/class/power_supply/BAT0/status /sys/class/power_supply/BAT0/capacity` | %80'in üstünde `Not charging` |

---

## 3. Sırada (Claude ile)

Faz 5 bitti (1 Kasım). Yol haritası değişti: tekne bir süre suya inemediği için **Faz 6 =
RoboBoat simülasyonda** (ARCHITECTURE.md §15–16).

| Adım | Ne | Durum |
|---|---|---|
| R1.1–R1.3 | 13 sınıf, ortak parkur okuyucu, gemiler ve iskeleler, `roboboat_2026` parkuru Nathan Benderson Park'ta | Bitti (commit `d951ade`) |
| R1.4–R1.5 | Kare başına tespitler (`/usv/detections`), kalıcı dünya haritası (`mapper`), RViz | Bitti (`79aca43`) |
| R1.6 | Sabit ölçüm rotası (`survey_drive`), `map_report.py`, ilk ölçüm (`survey_base`) | Bitti (`f98e8cc`) |
| R1.7 | Ölçüm zinciri düzeltmeleri, her biri ayrı ayar. D2 (derinlik → mesafe) + D1 (kameradan konumla): ölçüm zinciri haritası 1,74 → 0,28 m, hedef geçti. D4 (dönüş) + D5 (sınıf karışması, aynı karede görülme kuralı): `survey_r17`'de ort. 0,25 m / en fazla 0,42 m, onaylı hayalet 2. D6 (sınıf yarıçapı, ön yüz → merkez): `survey1_d6`'da ölçüm zinciri haritası ort. **0,14 m / en fazla 0,30 m**, onaylı kopya 3, onaylı hayalet 3; mutlak harita ort. 1,17 m (GPS kayması). D3 (GPS yaşı, 0,14 m) ertelendi: gerçek GPS hızı (`.param`) görülünce | **Bitti** (R1 kapandı) |
| **R2** | **Navigator'a yarışma profili (RoboBoat).** Kararlar (1 Kasım): aynı `navigator` düğümü, `competition` anahtarı; görev yerleri Görev 1 giriş kapılarına göre isteğe bağlı ipucu, yoksa keşif; tekne ilk kapının ≥1,8 m önüne kapıya bakarak konur. R2.1 iskelet (ortak taban `nav_base`, koşu yöneticisi, ROS bağlantıları) **bitti**. R2.2 parkur modeli (`roboboat_course.py`: Görev 1 kapıları + kapı çerçevesi, kanal + enkaz, Görev 3, marina iskeleleri, Görev 6 bölgeleri; `ws/tools/course_report.py`) **bitti**: `survey1_d6`'da GPS kusursuzken kapılar ve kanal 0,11–0,14 m, çerçeve 0,8°, ipuçları 0,24–0,61 m; teknenin kendi haritasında (GPS kayması) kapılar 0,8–1,5 m, çerçeve 4,6°, ipuçları 0,9–4,0 m. En iyi iskele yeri artık marinanın tamamı görülmeden verilmiyor. **R2.4 için not:** çerçevenin yönünü giriş sırasında kapılardan düz geçerken pusuladan düzelt (GPS'ten bağımsız). Sonra R2.3 hareketler (git, kapıdan geç, etrafında dön, ara), R2.4 Görev 1 giriş + çıkış simülasyonda | **Sıradaki iş** |
| R3 | Görevler: 1 → 2 → 3 → 6 → 5 → 4 | |
| R4 | RoboCommand raporlama (protobuf / TCP, 1 Hz heartbeat) | |
| R5 | Eşdeğerlik sonrası düzeltmeler (ARCHITECTURE §14 listesi), simülasyonda ölçülerek | |

Harita hedefi (31 Ekim kararı): ikiye ayrıldı.
- Ölçüm zinciri (GPS gerçeğe çekilmiş halde, 6 m içinden görülen onaylı izler): ortalama < 0,4 m, en fazla 0,8 m.
- Mutlak harita hatası: raporlanır, hedef yok. GPS alıcısının kayması belirliyor; RTK ya da görsel odometri gerekir.

Mimari kararlar ve tam yol haritası: `ARCHITECTURE.md`.

---

## 4. Altın testlerle kapsanmayan kısımlar

Bu satırlar eski kodla döngü döngü karşılaştırılmadı; bir taşıma hatası olursa ancak o yol çalışınca görülür.

| Kısım | Neden kapsanmadı | Ne zaman bakılır |
|---|---|---|
| Görev 3 çapa + devriye (`task3.py`: GOTO_STANDOFF, PATROL_1/2) | TEKNOFEST'e özgü; RoboBoat'ta değişecek (31 Ekim kararı) | TEKNOFEST'e tekrar gidilirse `task3_anchor` senaryosu yazılır |
| Röle tekrar denemeleri | Eski kodla karşılaştırmalı birim testi var (`test_relay.py`); gerçek uçuş kontrolcüsüyle değil | Faz 7, `fc_bridge` iskelede |

