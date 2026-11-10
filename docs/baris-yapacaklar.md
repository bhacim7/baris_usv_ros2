# Barış'ın yapacakları (benden beklenenler)

Kodda ya da simülasyonda varsaydığımız, ama gerçeğini Barış'ın vermesi, ölçmesi ya da yapması
gereken her şey burada. Barış "benden beklenen ne var?" diye sorunca Claude bu listeyi özetler.
Bir madde bitince işaretlenir ya da silinir; Claude her adımda yeni maddeleri ekler.

Son güncelleme: 10 Kasım 2026.

## Veri seti ve etiketleme
- [ ] **Sınıf adları simülasyondakiyle aynı olmalı** (`ws/src/usv_core/usv_core/classes.py`,
      liste alfabetik). Veri setinde başka ad kullanırsan haber ver, ikisini eşitleriz.
  - 2026'dan: `buoy_black|green|red|yellow`, `digit_1..3`, `indicator_green|red`, `plus`,
    `pole_green|red`, `triangle`.
  - 2027 eklenenler: `beacon_red|green|blue|off` (RoboBuoy gövdesi + ışık tek sınıf; sönük de
    etiketlenmeli), `buoy_blue`, `buoy_white` (Task 2 Polyform), `circle`, `square` (Task 4
    şekilleri; artı = `plus`, üçgen = `triangle`).
  - `window_red|green|blue|off` (marina pencereleri): simülasyonun çıktı adı. YOLO + OpenCV
    yolunu seçersek veri setine pencere değil **bina** sınıfı girer (ör. `dock_building`);
    karar senin.
- [ ] Yanıp sönen ışıklar: kırmızı/yeşil 1 s yanık 1 s sönük; sönük karelerdeki RoboBuoy'lar
      `beacon_off` diye etiketlenirse model gerçeği öğrenir.

## Algılama kararları ve ölçümler
- [ ] Marina penceresi ve bölme göstergesi için yöntem: YOLO tek başına mı, YOLO (bina) +
      OpenCV renk okuması mı (`~/Downloads/drone_color.py` deneyimi)? Önerim ikincisi.
- [ ] LED panelin kamerada nasıl göründüğünü dene (patlayıp beyaz görünüyor mu, pozlama).
- [ ] RoboBuoy'a ZED derinliği: kutunun ortası şamandıranın neresine denk geliyor (şu an
      0,24 m varsayıldı, `OBJECT_RADIUS_M`)? Gerçek bir şamandırayla ölçülecek.

## Donanım
- [ ] **Adım 8:** Marine'de iki ZED2i aynı AGX Orin'de: FPS ve GPU yükü ölçümü. Kaldırmazsa
      taret kamerasında derinliksiz ya da Logitech Brio.
- [ ] **Adım 7 için iki soru:** taretin sıfırlama için limit anahtarı var mı? `atis_sistem.py`'de
      `cleanup()` servoları 0'a çekince kalan toplar düşüyor mu?
- [ ] Hidrofon (Task 2 ileri seviyeler, pinger): alınacak mı? Karar verilmedikçe pinger
      simüle edilmiyor.
- [ ] Azure'a 3D LiDAR gelecek mi? (Şimdilik yokmuş gibi gidiyoruz.)
- [ ] Ubiquiti: teknelerin ve GCS'nin IP adresleri, ağ adı. Şimdilik GCS 192.168.1.10, Azure
      .11, Marine .12; portlar 15600/15601/15602 (`usv_nodes/config/boats/*.yaml`,
      `telemetry_bridge.yaml`).
- [ ] RFD doluluğu: iki tekne + drone aynı kanalda 0,3 s telemetriyle ölçülecek (gerçek
      telsizlerle).

## Takvim ve takım işleri
- [ ] **Proof of Readiness videosu** (Ocak 2027): gerçek teknenin suda iki kapı setinden otonom
      geçişi. Gerçek tekne işi (Faz 7) buna göre öne alınmalı.
- [ ] Mission Readiness belgeleri (TDR, site, poster, "Engineering in Action").
- [ ] 2027 el kitabı çıkınca `docs/roboboat-2027.md` §9'daki sorulara bakmak (ör. çadırda kendi
      RTK baz istasyonumuz serbest mi?).

## Simülasyonda varsaydıklarımız (el kitabı ya da sen düzeltirsen değişir)
- 2027 parkuru Primer görselinden, ölçek 0,08 m/piksel tahmin; Task 1 rastgele alanı 30×22 m.
- RoboBuoy panel tepe genişliği, elmas eni, köşe işaretçileri; Task 4 levhası 60 cm, şekil
  40 cm; Task 4 teknelerinin şekilleri; marina iskelesinin sudan yüksekliği.

## Her adımda
- [ ] Verdiğim test komutlarını çalıştırıp çıktıyı göndermek, sonra commit ve push.
