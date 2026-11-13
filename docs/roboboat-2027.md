# RoboBoat 2027: görevler, değişenler ve yol haritası

Kaynaklar (6 Kasım 2026'da okundu):
- **RoboBoat 2027 Primer & Task Ideas** (22 sayfa). 2027'nin tek resmî belgesi şimdilik bu.
  "Primer ve görev fikirleri" düzeyinde: puanlar, ölçüler, RoboCommand 2027 protokolü
  henüz yok. Ayrıntılı el kitabı (handbook) daha sonra çıkacak.
- **RoboBoat 2026 Resources** (geçen yılın el kitabı, 110 sayfa): RoboCommand, eleme, araç
  ve güvenlik kuralları, çoklu tekne. 2027'de değişebilir ama en iyi tahminimiz bunlar.
- **RobotX 2026 belgeleri**: RoboBuoy, Light Beacon (ışıklı işaretçi), Color Indicator (renk
  göstergesi), Docking Bay Structure (yanaşma yapısı). 2027 Primer'ın çizimlerindeki unsurlar
  bunlara çok benziyor; ölçüler ve davranışlar için en iyi kaynak şimdilik bunlar.

Bu belgede **kesin** olanlar Primer'da yazanlar. **Yorum** ya da **tahmin** olanlar öyle
işaretlendi; el kitabı çıkınca gözden geçirilecek.

---

## 1. Ne değişti (2026 → 2027)

| Konu | 2026 | 2027 |
|---|---|---|
| Tarih, yer | Şubat 2026, Nathan Benderson Park | **18–23 Şubat 2027**, yine Nathan Benderson Park |
| Tema | Storm Response (fırtına sonrası) | "Discover the Unknown"; ada limanı ve marina |
| Görev sayısı | 6 (kanal + 5) | **5 görev**, görevler "misyon"ların parçası |
| Puan yapısı | Görev başına puan | Görevler + **Mission Badge** (bir çalışmada belli görev/seviye bileşimini tamamlayınca rozet; sıralamaya katkı) |
| Elemeye giriş | Yerinde eleme turu (2 görev temel, 2 görev ileri, 2 görev art arda, heartbeat) | **Proof of Readiness**: yarışmadan önce gönderilen kanıt. Otonom geçiş videosu (iki kapı setinden geçiş, çarpma yok, iyi otonom kontrol) + güvenlik sistemleri foto/video (kill switch, uzaktan kumanda, çekme halatı bağlantısı, pervane koruyucuları) |
| İtki testi | Vardı | **Kaldırıldı** |
| Tasarım belgeleri | TDR, site, tanıtım videosu, sunum, sistem değerlendirme | **Mission Readiness**: 6 sayfalık TDR, site, 1 sayfalık "Engineering in Action", poster, 15 dk teknik mülakat, 15 dk sistem değerlendirmesi |
| Kayıt ücreti | — | 1500 $ (gözlemci takım 750 $) |
| Döngü | Yıllık | **İki yıllık**: 2027 alan yarışmaları, 2028'de ortak RobotX etkinliği |
| Tekne sayısı | En fazla 2 | En fazla 2 (değişmedi). 2027'de **iki teknenin birlikte çalışması görevlerin içine girdi** |
| RoboCommand | Yalnız raporlama (tekneden hakeme) | **Hakemden tekneye görev mesajları da var** (Task 4, Task 5). Protokol henüz yayımlanmadı |

**En önemli tarih:** Primer'daki takvimde yarışma öncesi teslimler "Ocak" diye geçiyor
(belgede "JAN 2026" yazıyor, büyük olasılıkla "Ocak 2027" demek istiyorlar). Proof of
Readiness videosu da bu teslimlerin içinde olmalı. Yani **gerçek teknenin suda otonom kapı
geçişini Ocak 2027'ye kadar videoya alması** gerekiyor. Bugünden yaklaşık 2 ay.

---

## 2. Parkurdaki unsurlar (neye benziyorlar)

### RoboBuoy (görev şamandırası)
- Yüzen, beyaz plastik (coroplast) kaplı bir **piramit**. Taban paneli 17 inç (~43 cm),
  panel yüksekliği 12 inç (~30 cm). Altında havuz köpüğünden yüzdürücü kare çerçeve.
- Dört yüzünde **siyah sembol**, türünü gösteriyor:
  - **elmas** (RobotX'te Task 1, parkur başına 10 adet),
  - **daire** (RobotX'te Task 2, 2 adet),
  - **harf** (parkur sınır işaretleri, 4 adet).
- Tepesine iki şeyden biri takılıyor:
  - **Light Beacon (ışıklı işaretçi):** ~9 cm çapında PVC boru, etrafında adreslenebilir RGB LED
    şerit (her renk, yanıp sönme, parlaklık ayarlı; ESP32 ile Wi-Fi'den kumanda). Üstünde
    güneşi kesen 25 cm'lik siyah şapka. İçinde yukarı bakan ayna koni (drondan görünsün diye).
  - **Color Indicator (renk göstergesi):** 3D yazıcıda basılmış silindir; servo kanatları
    döndürüp **yalnız kırmızı ya da yeşil** gösteriyor, 360° yatay görünür. 2026 RoboBoat'taki
    göstergenin aynısı (bizim simülasyonda zaten olan `indicator`).
- 2027 Primer çizimlerinde Task 1 şamandıralarının tepesinde **kırmızı, yeşil, mavi ve
  "siyah" (sönük)** ışıklar var; Task 2'de numaralı (1, 2, 3) RoboBuoy'ların tepesinde küçük
  kırmızı/yeşil işaretler.

### Polyform şamandıralar
- Bildiğimiz yuvarlak renkli şamandıralar. Task 2 çiziminde: yeşil, beyaz, kırmızı, mavi,
  sarı, siyah.

### Akustik pinger
- Suya ses yayan cihaz. Task 2'nin ileri seviyelerinde başlangıç/bitiş yerini **yalnız sesle**
  bulmak gerekiyor → teknede **hidrofon dizisi** gerekir (2026'nın Task 6 "Harbor Alert"
  ses görevinin yerini almış gibi).

### Marina ve yanaşma yapısı (Docking Bay Structure)
- Yüzer iskele küplerinden (50×50 cm) **3 yanaşma bölmesi**. RobotX ölçüsüyle iskele 6,6 m
  uzunluğunda, bölmeler ~1,5 m genişliğinde, ~2 m derinliğinde. (Bizim teknenin gövdesi
  0,78 m geniş, 1,7 m uzun: sığıyor ama her yanda ~0,35 m pay var.)
- Her bölmede **iki pencereli küçük bir "bina"**: pencereler 16×16 RGB LED panel (renk ve
  sıra gösterebilir). Bölmeye bakan tabanda bir **renk göstergesi** (yeşil = boş bölme).
- Binanın önünde **su toplama sistemi**: 4 litrelik bidon, akış sensörü; hedef ağzı
  21×28 cm. "Yangını söndürmek" = **suyu bu hedefe püskürtmek** (akış sensörü ölçüyor).
- 2027 marina çiziminde ayrıca bir iskeleye dizili numaralı (1, 2, 3) yapılar ve "distractor"
  (şaşırtmaca) tekneler var.

### Coroplast tekneler (Task 4)
- Sarı ya da siyah gövdeli küçük maket tekneler, "yelken" yerinde beyaz levha üzerinde
  **dört şekilden biri: artı (cruciform), daire, kare, üçgen**. Su teslimatı hedefi.

---

## 3. Görevler tek tek

Her görevin üç seviyesi var: **Core** (temel), **Advanced** (ileri), **Disruptive** (en zor).

### Task 1: Safe Passage (güvenli geçiş)
**Ne isteniyor:** Limandaki enkaz alanında, ENTRY (giriş) noktasından EXIT (çıkış)
noktasına güvenli bir yol bulmak. Yol her seferinde rastgele kuruluyor.
1. ENTRY ve EXIT noktalarını, ayrıca kırmızı, yeşil ve siyah (sönük, "OFF") şamandıraları
   bulup sınıflandırarak **raporlamak**.
2. ENTRY şamandırasının etrafında **saat yönünde** tam tur atmak.
3. Alandan geçerken **yeşilleri teknenin soluna (port), kırmızıları sağına (starboard)**
   alarak geçmek; hiçbir şeye değmemek.
4. EXIT şamandırasının etrafında **saat yönünün tersine** tur atıp çıkmak.

**Seviyeler:**
- Core: yukarıdaki gibi.
- Advanced: tekne alandayken **EXIT şamandırası değişiyor**.
- Disruptive: EXIT ve en fazla iki şamandıra daha **renk değiştiriyor**.

**Yorumumuz:**
- Çizimde ENTRY ve EXIT'te mavi ışıklı şamandıralar var; büyük olasılıkla **mavi =
  giriş/çıkış**. (Tahmin, el kitabında bakılacak.)
- Kapı yok; her şamandıra tek başına "beni şu tarafına al" diyor. Yani kanaldaki "kapının
  ortasından geç" yerine **her şamandıra için yan kısıtı olan bir yol planı** gerekiyor.
- Renkler LED ışık: güneşte, uzaktan ve **sönük/yanık** ayrımıyla tanınması gerekiyor. YOLO
  kutusu + kutunun içindeki renk/parlaklık ölçümü gibi bir çözüm gerekebilir.
- Dikkat: 2026 kanalımızda kırmızı **sol**daydı (bizim parkur dosyası). 2027'de kırmızı
  **sağ**da (ABD kuralı "red right returning" ile aynı).

**Elimizde olan:** harita, A* (görünürlük grafiği), Circle (saat yönü ve tersi), engelden
kaçınma, takılma bekçisi. **Yeni gereken:** yan kısıtlı yol planı, ışıklı şamandıra
algılaması (renk + sönük), değişen renge göre yeniden planlama, rapor mesajları.

### Task 2: Emergency Response Sprint (acil müdahale sürati)
**Ne isteniyor:** Bir noktadan yardım gereken bir alana hızla gitmek (örneğin denizde
zorda bir yüzücü). Doğru START şamandırasına git, **doğru renkteki** polyform şamandıranın
etrafından dolan, sonra bitiş şamandırasına **süratle** git.

**Seviyeler:**
- Core: YEŞİL şamandıradan başla, doğru hedefin etrafından dön, alandan çık.
- Advanced: aynı, ama **akustik pingeri çalışan şamandıranın yanından** çık.
- Disruptive: START ve END yalnız akustik pingerle bulunabiliyor, gözle görülemiyor;
  atanan hedefin etrafından dön, END'in yanından çık.

**Yorumumuz:** "Doğru renk" nereden öğreniliyor belli değil (RoboCommand mesajı mı,
başlangıçtaki bir gösterge mi?). Sürat önemli, muhtemelen süre puanı var. 2026'nın Task 3'üne
(hız parkuru) benziyor.

**Elimizde olan:** Circle, DriveTo, renk sınıfları. **Yeni gereken:** hızlı sürüş ayarları,
**hidrofonla ses kaynağı bulma** (ileri seviyeler için donanım + yazılım).

### Task 3: Navigate the Marina (marina)
**Ne isteniyor:** Marinaya gir, **boş bölmeye** (yeşil gösterge) yanaş, bir iş yap.

**Seviyeler:**
- Core: yeşil göstergeli bölmeye yanaş.
- Advanced: yanaştıktan sonra **kırmızı ışıkla gösterilen "yangını söndür"**: pencerede
  kırmızı yanan hedefe su püskürt.
- Disruptive: ardından bölmedeki bir pencere **iki renkli bir ışık dizisi** yakar
  (kırmızı ya da mavi):
  - 1. renk ne teslim edileceği: **su (kırmızı)** ya da **raketbol topu (mavi)**;
  - 2. renk hangi distractor tekneye teslim edileceği.

Primer, iki teknenin bu işleri **ekip olarak** yapabileceğini söylüyor.

**Elimizde olan:** parkur modeli iskeleleri ve göstergeleri tanıyor (2026 Task 5 için
yazılmıştı), DriveTo, A*. **Yeni gereken:** dar bölmeye yanaşma (hassas manevra, durma),
**su topu (pompa + nozul + nişan)**, **top fırlatıcı**, LED panel renk/sıra okuma.

### Task 4: Delivery Run (teslimat)
**Ne isteniyor:** RoboCommand'dan gelen mesaj, belli bir şekilli (artı, daire, kare, üçgen)
coroplast tekneye **su teslim etmeyi** istiyor. Bu **acil** bir istek: tekne yaptığı işi
hemen bırakıp teslimatı yapmalı, sonra kaldığı yerden devam etmeli.

**Seviyeler:** Core 1 tekneye, Advanced 2 tekneye, Disruptive 3 tekneye su.

**Yorumumuz:** "Su teslim etmek" büyük olasılıkla Task 3'teki gibi su püskürtmek (2026
Task 4 "Supply Drop"a benziyor). İki büyük yazılım gereksinimi var: **RoboCommand'dan mesaj
almak** ve görevin ortasında **araya girip sonra devam edebilen bir görev yöneticisi**.

**Yeni gereken:** RoboCommand'dan gelen mesajı okumak, görev kesme ve devam etme,
levhadaki şekli tanıma, su topu.

### Task 5: Dynamic Response (dinamik müdahale)
**Ne isteniyor:** Başka bir iş yaparken RoboCommand'ın beklenmedik görevlendirmesine anında
cevap vermek.

**Seviyeler:**
- Core: işi bırakıp **istenen GPS noktasında toplan**.
- Advanced: **iki tekne birlikte**, koşunun başında RoboCommand'ın istediği bir Mission
  Badge'i yapar.
- Disruptive: aynı, ama istek tekneler başka işlere başladıktan sonra gelir.

**Yorumumuz:** Core kolay sayılır (RoboCommand mesajı + DriveTo). Advanced ve Disruptive
**iki tekne olmadan yapılamıyor**.

### Communications & Reporting (bütün görevler)
"Bütün tekneler RoboCommand ile etkileşebilmeli." Raporlama her görevin içinde.

---

## 4. Mission Badge'ler

Üç rozet var. Her biri için aynı üç görevin belli seviyelerde tamamlanması isteniyor:
**Safe Passage + Navigate the Marina + Dynamic Response**. Seviyeler henüz verilmedi.

| Rozet | Ölçtüğü yetenekler |
|---|---|
| Assessment and Discovery | Navigasyon, görsel tanıma, engelden kaçınma |
| Automated Logistics Support | Navigasyon, raporlama, görsel tanıma, altyapı etkileşimi (yanaşma, teslimat), iki adımlı davranışlar |
| Dynamic Tasking | Raporlama, altyapı etkileşimi, iki adımlı davranışlar; RoboCommand görevlendirmesine cevap |

**Yorumumuz:** Rozetlerin ortak noktası Task 1, 3 ve 5. Bu üçü öncelikli olmalı.

Primer'da ayrıca 2027'de değerlendirilecek davranışlar listeleniyor: navigasyon, engelden
kaçınma, **station keeping (yerinde durma)**, **akustik konum bulma**, raporlama, görsel
tanıma, altyapı etkileşimi, iki adımlı davranışlar.

---

## 5. RoboCommand nedir?

Hakemlerin bilgisayar sistemi. Teknenin ne gördüğünü ve ne yaptığını **hakeme bildirdiği**
kanal. 2026'da böyle çalışıyordu (2027 için henüz yayımlanmadı):

```
tekne ──(kendi telsiz/Wi-Fi bağlantın)──> kıyıdaki bilgisayar (OCS, operasyon çadırında)
      ──(çadırdaki RJ-45 Ethernet kablosu)──> RoboCommand sunucusu 10.10.10.1, TCP port 50000
```

- Senin hatırladığın **Ethernet bağlantısı** bu: kıyı bilgisayarı çadırdaki kabloya takılıyor,
  DHCP ile 10.10.10.100–200 arası bir adres alıyor. İnternet yok, kapalı ağ.
- **Mesaj biçimi:** Protocol Buffers (protobuf). RoboNation `report.proto` dosyasını, Python ve
  C++ örneklerini GitHub'da (`robonation/robocommand`) yayımlıyor. Her mesaj
  `$R` + 1 bayt uzunluk + protobuf içerik + `!!`.
- **Heartbeat:** 1 Hz "yaşıyorum" mesajı. 2026'da koşu bununla başlıyordu ve elemenin şartıydı.
- **Zaman damgası:** her mesajda olmalı; saat, sahadaki NTP sunucusuna (10.10.10.1)
  eşitleniyor. En fazla 5 mesaj/saniye.
- **2026 görev mesajları:** GatePass (kapı geçişi), ObjectDetected (nesne bulundu: tür, renk,
  konum), ObjectDelivery (teslimat), Docking (yanaşma), SoundSignal (ses).
- 2026'da veri **yalnız tekneden hakeme** akıyordu, ama tekne hakeme doğrudan bağlanmıyordu:
  tekne verisi GCS'ye geliyor, GCS'ye takılı Ethernet kablosuyla RoboCommand'a aktarılıyordu
  (Barış, 11 Kasım). 2026 kuralına göre tekne otonomken kıyıdan kontrol
  bilgisi alamaz, yalnız rapor gönderir.

**2027'de değişen:** Task 4 ve Task 5'te **hakem tekneye görev gönderiyor**. Yani bağlantı iki
yönlü olacak. Bunun nasıl olacağı (hangi mesajlar, kıyı bilgisayarı üzerinden mi) henüz
yayımlanmadı. Mimarimizi buna hazır tasarlamalıyız. Örnek bir yapı: teknede bir
`robocommand_bridge` düğümü, kıyıda bir aktarıcı; simülasyonda da sahte bir RoboCommand
sunucusu.

---

## 6. İki tekne ne demek?

**Kurallar** (2026; 2027'de büyük olasılıkla benzer):
- Her tekne ayrı ayrı boyut (1,8 × 0,9 × 0,9 m kutu) ve ağırlık (tüm sistem < 140 lb ≈ 63 kg)
  sınırına uyar; her birinin iki kill switch'i, uzaktan kumandası, çekme bağlantısı olur.
- Koşu başında her tekne önce giriş kapısından geçer (2026'da).
- Süre "sürü" gibi işler: biri kıyıya alınsa da diğeri sudayken süre akar.
- **Inter-Vehicle Communication (IVC)** ek puanı (2026): tek yönlü mesaj (temel), diğer teknenin
  onayı ya da tepkisi (ileri), iki yönlü koordineli davranış (en zor; örneğin iki teknenin
  iki boş bölmeyi bulup aynı anda yanaşması). Mesajlaşırken ışıkla göstermek ek puan.
  Kullanılan frekansı Discord'da duyurmak gerekiyor; koşu sonrası kayıtlarla kanıt.

**2027'de iki teknenin anlamı:**
- Task 5'in Advanced ve Disruptive seviyeleri **iki tekne olmadan yapılamıyor**.
- Task 3'te iki teknenin işleri bölüşmesi öneriliyor.
- Rozetler büyük olasılıkla iki tekneyle daha kolay toplanıyor.

**Yazılım için anlamı:**
- **Aynı yazılım iki teknede**, ROS 2'de ayrı isim alanlarında (`/usv1/...`, `/usv2/...`) ya da
  ayrı domain'lerde. Bizim düğümler buna şimdiden uygun yazılmalı.
- **Tekneler arası bağlantı:** konum, harita ve görev paylaşımı; kim ne yapacak (görev dağıtımı).
- **Birbirine çarpmamak:** her tekne diğerini hareketli bir engel olarak görmeli (paylaşılan
  konumla ya da kamerayla).
- **RoboCommand:** her tekne kendi raporunu gönderir; görevlendirmeler hangi tekneye gidiyor,
  bakılacak.
- **Simülasyon:** Gazebo'da iki tekne (`sim.launch.py`'yi buna açmak).

**Donanım için anlamı:** iki tam tekne (iki Jetson, iki sensör seti, iki kill switch sistemi)
ya da biri daha basit ikinci bir tekne. Bu, bütçe ve zaman açısından takımın kararı.

### Bizim iki teknemiz: Azure ve Marine (Barış, 6 Kasım)

| | **Azure** (USV1) | **Marine** (USV2) |
|---|---|---|
| Görev bilgisayarı | Jetson AGX Thor | Jetson AGX Orin (elimizde) |
| Atış sistemi | yok | top fırlatıcı + su topu, 40-pin header'dan; tarette kendi ZED2i'si |
| 3D LiDAR | gelirse buna (şimdilik yokmuş gibi) | yok |
| Aynı olanlar | gövde, iticiler, Orange Cube, GPS, ZED kamera, bataryalar | aynı |

- Atış sistemi 2026'da ayrı bir Orin Nano'daydı (`atis_sistem.py`). 2027'de teknenin kendi
  bilgisayarında çalışacak, yani Marine'in iki gözü olacak: seyir kamerası ve taret kamerası.
- Bütün sınıflar tek veri setinde, iki tekne aynı modeli kullanıyor. Light beacon'lar da
  bugünkü indicator'lar gibi **gövde + ışık birlikte** etiketlenecek: ışığın rengi sınıfın
  içinde (örneğin `beacon_red`, `beacon_off`; adlar veri setiyle kesinleşir).
- ROS isim alanları teknelerin adıyla: `/azure/...`, `/marine/...`.

### Haberleşme (Barış, 6 Kasım)

```
RoboCommand (10.10.10.1:50000) ──Ethernet── GCS ──Ubiquiti (ana)──> Azure, Marine
                                                 ──RFD (yedek)────> Azure, Marine
```

- 2026'da GCS, RoboCommand'ın verdiği Ethernet kablosuna bağlıydı ve tekneden gelenleri
  aktarıyordu. Teknelere komut yalnız RFD ile gidiyordu.
- 2027'de RoboCommand'dan gelen görevler de bu kablodan GCS'ye gelecek (büyük olasılıkla). GCS
  bunları teknelere hem Ubiquiti hem RFD ile aktaracak.
- **Yazılım için anlamı:**
  - RoboCommand bağlantısı (heartbeat, protobuf, NTP) **GCS'de** çalışır.
  - Teknede bir **bağlantı düğümü**: raporları GCS'ye gönderir, görevleri alır.
  - GCS ile tekneler arasında kendi küçük mesajlarımız (UDP/TCP). ROS'un DDS trafiğini
    Wi-Fi'a açmıyoruz (önerimiz: daha az yük, bağlantı kopunca sorun çıkmaz).
  - Aynı mesaj iki yoldan gelebilir: mesaj numarasıyla tekrarlar ayıklanır. Görev mesajları
    RFD'ye sığacak kadar küçük tutulur.
  - GCS **yalnız RoboCommand'dan gelenleri** aktarır ve hepsini kaydeder (otonomken kıyıdan
    kontrol yasak; hakem sorarsa kanıt).
  - Tekneler arası mesajlar (IVC) da bu ağdan geçebilir.
- RFD teknede **Jetson'a** bağlı (Barış): teknedeki bağlantı düğümü RFD'yi seri porttan,
  Ubiquiti'yi ağdan dinler.
- GCS yazılımı: 2026'nın `GCSv1000.py`'si (PySide6). Bugün tek seri port (RFD, 57600), JSON
  satırları, sıra numaralı güvenilir komut kanalı (`cq` / `aq`). İçinde iki tekne için yer
  kısmen var (IDA 1 / IDA 2 görev tabloları, iz, tekne kimliği `i`), ama ekranda yalnız
  IDA 1 paneli var.

---

## 7. Bugüne kadar yaptıklarımızdan neler işe yarıyor?

| Bizde olan | 2027'de |
|---|---|
| ROS 2 yapısı, Gazebo + VRX simülasyonu, raporlama ve yeniden oynatma araçları | Aynen. Parkur dosyaları 2027 unsurlarıyla yenilenecek |
| Kalıcı dünya haritası (D4–D7 temizlikleri) | Aynen; R3.5 (GPS kayması tahmini) hâlâ önemli |
| Hareketler: DriveTo (A*), Circle (saat yönü/tersi, çember genişlemesi), Search, takılma bekçisi | Task 1 (giriş/çıkış turu, engel), Task 2 (hedef etrafında dönüş), Task 5 (GPS noktasına git) doğrudan kullanır |
| PassGate, kanal, enkaz alanı (2026 Task 2) | Proof of Readiness videosundaki **kapı geçişi** için PassGate; enkaz taraması Task 1'e fikir veriyor |
| Task 1 giriş ve çıkış kapıları | Proof of Readiness'taki "iki kapı setinden geçiş" ile büyük olasılıkla aynı |
| Parkur modelinde iskele ve gösterge tanıma (2026 Task 5 için) | Task 3'ün başlangıcı |
| 13 sınıflı algılama | **Yeni sınıflar gerekecek:** ışıklı RoboBuoy (renk + sönük), elmas/daire/harf panelleri, numaralı şamandıralar, polyform renkleri, şekilli coroplast tekneler, LED pencere. YOLO veri seti ve eğitimi yeniden |
| TEKNOFEST kodu | Dokunulmuyor (altın testlerle korunuyor) |

---

## 8. Yol haritası (6 Kasım, Barış'la; 14 Kasım'da görev davranışlarıyla güncellendi)

**Zaman çizelgesi:** bugün 14 Kasım 2026 → teslimler (Proof of Readiness videosu) Ocak 2027 →
yarışma 18 Şubat 2027.

**Durum (14 Kasım):** Adım 1–6 bitti: altyapı tamam (iki tekne, parkur, 2027 unsurları,
haberleşme, RoboCommand, görev yöneticisi, paylaştırma). Görev davranışlarının hiçbiri yazılmadı
(görev yöneticisi "not written yet" deyip atlıyor); yalnız `rally`, `hold` ve eve dönüş var.

**Karar (Barış, 14 Kasım):** sıradaki iş **görev davranışları, Gazebo'da** (Adım 9–12). Barış'ın
sınav haftası: boş zamanlarda simülasyonda ilerlenir, gerçek tekne (Faz 7) biraz bekler.
- Öncelik yine Faz 7'yi video için yetiştirmek. Tekne mekanik olarak hazır (14 Ekim videosundaki
  gibi yüzüyor).
- Yetişmezse yedek: video tekneyi süren eski kodla (2026 multiprocess) çekilir, ROS 2 sisteminden
  küçük düzeltmelerle.
- Faz 7'de yazılacaklar (o zaman ayrıca planlanır): `fc_bridge` (MAVLink, Orange Cube),
  `perception` (ZED + YOLO), teknede açılış (`boat.launch.py`), iskele testleri, suda readiness
  koşusu ve video.

**Şimdilik yapılmayacaklar:** gerçek tekne (Faz 7), MQTT, pinger (hidrofon kararı yok), 3D
LiDAR, su ya da top isteyen seviyeler (Task 3 Advanced/Disruptive, Task 4; Adım 7'den sonra).

### Adım 1. İki tekneye hazır kod (isim alanları)
- 8 dosyadaki 57 mutlak konu ve servis adı göreli olur (`'/usv/fix'` → `'fix'`).
- `sim.launch.py`, `boat:=azure|marine` argümanını alır (varsayılan `azure`). Düğümler
  `/azure/...` altında çalışır, frame adlarına önek gelir.
- Araçlar (replay, bag_read, raporlar) isim alanını ayar olarak alır. Eski kayıtlar
  (`/usv/...`) `--ns usv` ile okunur. Belgelerdeki komutlar güncellenir.
- Davranış değişmez; testler ve bir simülasyon koşusu bunu gösterir. TEKNOFEST mantığına
  dokunulmaz.
- Frame adlarına önek şimdilik yok: TF'yi yalnız RViz kullanıyor, tek tekne için. Önekler iki
  tekneli simülasyonla, iki teknenin ortak bir çerçevesiyle gelir.

### Adım 2. Tekne profilleri
- İki dosya: `ws/src/usv_nodes/config/boats/azure.yaml` ve `marine.yaml`. İçlerinde tekne
  numarası (`number`, GCS.deki `i`), bilgisayar, kameralar, LiDAR, su topu ve top sayısı var.
- `usv_core/boat.py` dosyayı okur; `can('water')` gibi sorulara cevap verir.
- Launch profili okur ve açılışta yazar (`Boat marine ... can: water, balls, turret_camera`).
  Görev yöneticisi "su sıkabilen tekne Marine" bilgisini buradan alacak.
- Yazıldı (10 Kasım).

### Adım 3. Tek parkur: roboboat_2027 (2026 Task 1–2 + 2027 parkuru)
- `roboboat_2026.yaml` dosyasının adı `roboboat_2027.yaml` olur.
  - 2026'nın Task 1 ve Task 2'si kalır: Proof of Readiness videosu için.
  - Task 3–6 unsurları (marina, sarı şamandıralar, tekneler) çıkar.
- 2027 parkuru aynı haritaya kurulur. Parktaki yerini Barış Gazebo'da seçer.
- İki başlangıç noktası:
  - `start:=competition` (varsayılan): 2027 parkurunun başı.
  - `start:=readiness`: eski Task 1'in önü.
- Navigatörde görev seçimi:
  - `mission:=roboboat_2027`: 2027 görevleri.
  - `mission:=readiness`: bugünkü Task 1 + Task 2.
  - 2026 görev kodu silinmez.
- **3a yazıldı (10 Kasım):** dosyalar `roboboat_2027.yaml` ve `roboboat_2027_far.yaml`.
  - Kalan: Task 1 direkleri, Task 2 kanalı ve enkaz alanı, enkazdaki siyah tekne.
  - Dosyada `starts` ve `missions` bölümleri; `course.select` birini seçer. Şimdilik yalnız
    `readiness` var ve varsayılan o. `competition` ve `roboboat_2027` yer seçilince ve
    Adım 6'da eklenir; varsayılan o zaman değişir.
  - Survey rotası kısaldı (~130 m, Task 2'den sonra başa döner): eski harita ölçümleriyle
    sayılar birebir karşılaştırılamaz.
  - Eski tam parkur: `ws/src/usv_sim/test/data/roboboat_2026_full.yaml` (iskele ve tekne
    testleri onu okur) ve git geçmişi (`4cf5496`).
- **3a ölçüldü (10 Kasım, `r27_s1`):** FINISHED 265 s, kapılar ve kanal temiz, temas 0.
  Video için dikkat: yeşil 349° döndü (haritada 0,16 m doğru yerde olduğu halde; kayma
  açıklaması burada yetmiyor), bir siyah şamandıraya en yakın geçiş +0,02 m.
- **3b yazıldı (10 Kasım):** `competition` başlangıcı Gazebo (-197,80; 809,98), batıya bakar
  (eski parkurla aynı yön; Barış seçti). Readiness başlangıcının ~137 m güneyi. Parkur boş;
  unsurlar Adım 4'te bu noktanın önüne çizilir. Varsayılan şimdilik `readiness` kalıyor: boş
  bir parkurda başlamak işe yaramaz; Adım 4'te unsurlar gelince `competition` olur.
- `roboboat_2026_far.yaml` yeni dosyanın bir varyantı olarak kalır. Testler, survey rotası ve
  raporlar yeni dosyaya göre güncellenir. Eski kayıtların gerçek konumlarla karşılaştırılması
  için eski dosya git geçmişinde duruyor.

### Adım 4. 2027 unsurları ve ışıklar (simülasyon)
- `course.py`'ye yeni unsurlar:
  - RoboBuoy: piramit, üstünde elmas, daire ya da harf panelleri.
  - Light Beacon: renk, sönük, yanıp sönme ve bir zaman çizelgesi (örneğin "tekne alana
    girince EXIT kırmızı olur").
  - Polyform şamandıralar.
  - Yanaşma yapısı: iki LED pencere, renk göstergesi, su hedefi.
  - Şekilli coroplast tekneler.
  - Pinger: şimdilik yalnız konum bilgisi.
- `fake_perception` sınıf adını ışığın o anki durumundan üretir (`beacon_red`, `beacon_off`
  gibi), Barış'ın veri seti gibi gövde + ışık tek sınıf olarak.
  - Yanıp sönmede bazı karelerde `off` döner.
  - Harita bir şamandıranın rengini tek bir kareye bakarak değiştirmemeli.
- Gazebo'da ışığın renginin değişmesi yalnız görsel bir ayrıntı; nasıl yapılacağı
  araştırılacak.
- Task 1 alanı için seed'li rastgele parkur üreteci: her koşuda farklı düzen, ama her zaman
  geçerli bir yol var.
- Sıra: Task 1, Task 3 marina, Task 5 toplanma, Task 2, Task 4.
- **4a (Task 1) yazıldı (10 Kasım):**
  - Sınıflar `beacon_red`, `beacon_green`, `beacon_blue`, `beacon_off` (Barış: şimdilik bu
    adlar, veri seti değişirse haber verecek). Liste alfabetik; eski sınıfların numaraları
    kaydı, kod adla çalışıyor.
  - VRX'te 2026 RoboBuoy modeli yok (yalnız eski `robotx_light_buoy_*`, 2022 iskele ve
    levhalar). Model RobotX 2026 yapım kılavuzunun ölçüleriyle üretildi
    (`usv_sim/robobuoy.py`, `scripts/make_robobuoy_mesh.py`): 17×12 inç paneller, 8 inç
    elmas, 3,5 inç ışık borusu, 10 inç şapka, havuz köpüğü çerçeve. Panellerin tepe genişliği,
    elmasın eni ve köpüğün rengi tahmin.
  - Işık kuralı (RobotX 2026 Task 1): kırmızı, yeşil ve ENTRY mavisi 1 s yanık, 1 s sönük;
    EXIT mavisi sabit; sönük = engel. Çizimlerdeki "iki mavi farklı" sorusunun cevabı bu.
    `fake_perception` her karede ışığın o anki halinden sınıf üretir (yanıp sönen ışık sönük
    karelerde `beacon_off`). Gazebo ışığı başladığı renkte gösterir (Barış'la karar).
  - Harita: dört sınıf tek tür, `beacon`; renk son 2,5 s'deki yanık görülmelerden okunur,
    "sönük" yalnız o sürede hiç yanmadıysa.
  - Alan üreteci (`usv_sim/task1_field.py`): 10 şamandıra, 40×25 m, aralık ≥ 4 m (alan
    boyutu tahmin, el kitapları vermiyor). Önce ENTRY'den EXIT'e rastgele bir yol çizilir,
    yeşiller soluna, kırmızılar sağına konur: her düzende geçerli bir yol var. `layout:=<n>`
    ile başka düzen, `level:=advanced` ile EXIT tekne alanın ortasına gelince yer değiştirir,
    `level:=disruptive` ile ayrıca en fazla iki kırmızı/yeşil renk değiştirir.
  - Varsayılan başlangıç hâlâ `readiness`: 2027 görevi (Adım 6) yazılınca `competition` olur.
- **4b (marina) ve Primer yerleşimi yazıldı (10 Kasım):**
  - 2027 parkuru Primer'ın "Course layout" görseline göre kuruldu: görseldeki sol alttaki
    siyah tekne bizim `competition` başlangıcımız, görselin yukarısı ileri (parkta batı).
    Görsel ölçekli değil: 0,08 m/piksel tahmin (Task 1 şamandıraları ~10 m arayla, parkur
    ~86×70 m). Unsurlar gerçek boyutta, çizildikleri yerin ortasında.
  - Task 1: `layout: primer` görseldeki on şamandıra (ışınlı mavi = ENTRY yanıp sönen, ışınsız
    mavi = EXIT sabit); `layout:=<n>` rastgele alan (aynı bölgede, ENTRY'den doğuya 30×22 m).
  - Marina (`usv_sim/marina.py`), ölçüler RobotX 2026 "Docking Bay Structure" çizimlerinden
    (Barış'ın PDF'i): 500 mm küpler, 13×2 güverte, 4 küplük parmaklar (bölme 1,5×2,0 m),
    bina 1,0×0,75×1,0 m, pencereler 210×290 ve 280×360 mm (siyah çerçeve, arkada LED panel),
    tabanda renk göstergesi. Bölme 2 yeşil, bölme 3'te tekne var (kırmızı).
  - Pencere ışığı: tekne yeşil bölmede 3 s durunca pencere 2 kırmızı; `fire_out` olayı
    (şimdilik elle: `ros2 topic pub --once /azure/sim/event std_msgs/msg/String "{data:
    fire_out}"`, Adım 7'de taret) gelince 5 s yeşil, sonra sönük; `level: disruptive` ile
    ardından 60 s iki renkli dizi (1. renk 1 s, sönük 1 s, 2. renk 1 s, sönük 2 s).
  - Yeni sınıflar: `window_red|green|blue|off`, `buoy_blue`, `buoy_white` (Task 2 Polyform).
    Haritada pencereler `window` türü, renk ışıklar gibi zaman penceresinden.
  - Görselden ayrıca: Task 2'nin numaralı RoboBuoy'ları (tepede renk göstergesi, yüzlerde
    numara: `digit_1..3`) ve altı Polyform şamandıra, dört köşe işaretçisi (harfli; kamera
    sınıfı yok), Task 4 tekneleri (şimdilik eski sarı/üçgen, siyah/artı; şekiller 4e), Task 5
    bölgesi (`zones`, suda bir şey yok). Pinger yok.
  - Barış Gazebo'da baktı (10 Kasım): yerleşim ve marina sorunsuz. Commit `338b325`.
  - Algılama yöntemi (Barış'ın sorusu): pencere ve bölme göstergesi için YOLO + OpenCV renk
    okuması düşünülüyor (`~/Downloads/drone_color.py` deneyimi). Simülasyon algılamanın
    sonucunu (`window_red` gibi) üretir; yöntem algılama tarafının kararı, navigasyon
    değişmez.
- **4e (Task 4 tekneleri) yazıldı (10 Kasım):** 2027 teknesi, gövdenin ortasında yelken gibi
  duran beyaz bir levha ve iki yüzünde siyah şekil: `plus` (artı), `circle`, `square`,
  `triangle` (yeni sınıflar `circle`, `square`). Levha 60 cm, şekil 40 cm (tahmin). Parkurdaki
  dört tekneye birer şekil (görselde görünmüyor, tahmin), marinadaki teknede daire.
- **Adım 4'ten kalan:** pinger (Task 2 ileri seviyeler) simüle edilmedi: hidrofon kararı
  yok. Gerekirse yalnız konum ve frekans bilgisi eklenir.

### Adım 5. RoboCommand ve haberleşme
- **GCS tarafı:** RoboCommand bağlantısı.
  - TCP, `$R` + uzunluk + içerik + `!!`.
  - 1 Hz heartbeat, en fazla 5 mesaj/s, NTP.
  - Kodlayıcı ayrı bir dosya; 2027 protokolü gelince yalnız o değişir.
- **Tekne tarafı:** bağlantı düğümü. ROS içinde mesajlarımız sabit kalır:
  - giden raporlar: heartbeat, kapı, nesne, yanaşma, teslimat;
  - gelen `TaskRequest`: tür, parametreler, hangi tekne.
- **GCS ↔ tekne:** Ubiquiti ana yol, RFD yedek. Aynı mesajın iki kopyası ayıklanır.
- **Simülasyon:**
  - Sahte RoboCommand sunucusu: mesajları çözer, heartbeat'i denetler, parkur dosyasındaki
    zamanlamayla görev gönderir.
  - Sahte GCS aktarıcısı.
- **GCS uygulaması (`GCSv1000.py`)** repoya alınır ve düzenlenir:
  - harita küçülüp ortaya gelir; sağa, soldaki IDA paneli gibi ikinci bir panel (Marine)
    eklenir; iki teknenin verileri ayrı ayrı görünür;
  - İHA kısımları kapatılır (silinmez: başka yarışmalarda gerekebilir);
  - RFD'nin yanına Ubiquiti bağlantısı ve RoboCommand aktarıcısı eklenir.

**Kararlar (Barış, 10 Kasım):**
- Komutlar ve telemetri **iki hattan da** gider; RFD yalnız yedek değil, aynı veriyi taşır.
  Tekne her komutu bir kez uygular (numarasından), GCS her teknenin en yeni telemetrisini
  gösterir.
- Ubiquiti üzerinde UDP (TCP değil): kayıp paketin arkasında kimse beklemez, bağlantı kurmak
  gerekmez; komutların kendi yeniden gönderimi var.
- GCS ROS'suz, düz Python + PySide6 (ROS bir şey katarsa konuşulur). Görseller (`background.png`,
  `halat.png`, `roboboat.jpg`) depoya girer.
- Adresler şimdilik: GCS 192.168.1.10, Azure .11, Marine .12; portlar 15600 / 15601 / 15602.
- RFD'de telemetri 2026'daki gibi her tekne için 0,3 s; ölçülecek.

**5a yazıldı (10 Kasım): iki hatlı GCS bağlantısı.**
- `usv_core/gcs_link.py` (ROS'suz, kütüphanesiz; teknede ve GCS'de aynı kod): 2026 protokolü
  (kısa anahtarlar, `cq`/`aq`, `q`), satır toparlama, tekne tarafında kopya ayıklama
  (`CommandFilter`), GCS tarafında en yeni telemetri (`NewestFilter`), tekne başına numaralı
  yeniden gönderim (`ReliableSender`), UDP ve seri hat (pyserial'siz).
- Teknede `telemetry_bridge` düğümü: iki hattı dinler, komutları navigatörün servislerine
  çevirir, telemetriyi iki hattan yollar.
- Simülasyon: `sim.launch.py link:=true` (UDP bu bilgisayarda, RFD yerine
  `scripts/fake_radio.py`: kanal hızında, isteğe bağlı kayıplı sahte telsiz).
- `ws/tools/gcs_cli.py`: GUI gelene kadar metin GCS (izle, komut gönder; hat seçilebilir).

**5b yazıldı (10 Kasım): GCS depoda (`gcs/gcs.py`, `gcs/README.md`).**
- 2026 `GCSv1000.py`'den: solda IDA 1 Azure, sağda IDA 2 Marine paneli, harita ortada; komutlar,
  röle, hat istatistiği, klavye odağı tekne başına.
- Tek bağlantı iş parçacığı (`LinkWorker`), iki hat (`gcs_link`); Windows'ta RFD pyserial ile.
- 2026 haritasında ikinci teknenin işaretçisi ve izi hiç oluşturulmamıştı; eklendi.
- İHA kutusu gizlendi; harita listesinde yalnız görseli olan haritalar.
- ROS'suz; ana makinede sanal ortamda PySide6 ile çalışır.

**5c yazıldı (11 Kasım): iç görev mesajı.** `usv_interfaces/TaskRequest` (kaynak, görev adı,
acil mi, konum, anahtar/değer parametreleri). GCS hattında yeni komut `task`; `telemetry_bridge`
onu `task_request` konusuna yayınlar ve onaylar. Dışarıdaki protokol (RoboCommand 2027)
ne olursa olsun ROS içinde bu mesaj sabit kalır; görev yöneticisi (Adım 6) bunu dinleyecek.

**5d yazıldı (11 Kasım): RoboCommand, GCS tarafı (`gcs/robocommand/`).**
- RoboNation'ın resmi deposundan (`robonation/robocommand`, Apache 2.0) RoboBoat 2026
  `report.proto` ve üretilmiş kod. RobotX 2026 (kardeş yarışma) MQTT ile iki yönlü:
  hakemden OCS'ye `command` (koşu başlat, Task 4) ve `course`; RoboBoat 2027 büyük olasılıkla
  buna benzeyecek. MQTT taşıması 2027 el kitabı isterse eklenir (test için ayrıca bir MQTT
  sunucusu gerekiyor).
- Katmanlar: çerçeve (`$R` + uzunluk + içerik + `!!`), TCP (kendiliğinden yeniden bağlanır),
  kodlayıcı (heartbeat ve raporlar), köprü (her tekneye 1 Hz heartbeat, yalnız duyulan
  teknelere; en fazla 5 mesaj/s, önce heartbeat; tekne başına sıra numarası).
- GCS'de RoboCommand kutusu; kuralları denetleyen sahte sunucu.
- Teknenin rapor üretmesi (kapı, nesne, yanaşma) görev yöneticisiyle (Adım 6) gelir. Hakemden
  gelen görevler için yol hazır (`TaskRequest`, 5c); 2026 protokolünde böyle mesaj yok.

### Adım 6. Görev yöneticisi
- Sıradaki görevlerin yanında, acil görevle araya girip sonra kaldığı yerden devam etme
  (Task 4, Task 5).
- Görevleri profile göre tekneye dağıtma (su işi Marine'e).

**Kararlar (Barış, 11 Kasım):**
- Readiness koşusu (2026 Task 1 + 2, yalnız Proof of Readiness videosu için) eski
  `RoboBoatRun`'da **dondurulur**; görev yöneticisi ona dokunmaz. Navigatör açılışta
  `mission`'a bakar: `readiness` → eski koşu, `roboboat_2027` → görev yöneticisi. Video günü
  açılış komutuna `mission:=readiness` yazılır; varsayılan (Adım 6 bitince) yarışma.
- Askıya alınan görev baştan başlar, koşunun ortak hafızasıyla (yaptığını atlayabilir).
- Tekneler arası paylaştırmayı GCS yapar, otomatik: RoboCommand'dan gelen isteği kendisi
  çözüp tekneye iletir; operatör dokunmaz.
- Plan bitince ya da süre dolunca: başlangıç noktasına dön, orada dur (ayarlanabilir).
- Gazebo'da iki tekne 6e'de. İki tekne şu an tamamen aynı, aynı kodu çalıştırır; ileride
  birine LiDAR, birine atış sistemi gelir.

Parçalar: 6a çekirdek, 6b bağlantı (navigatör `task_request`'i dinler, plan parkur
dosyasından, durum GCS'de), 6c `rally` ve `hold`/eve dönüş (yerinde durma), 6d rapor yolu
(tekne → GCS → RoboCommand), 6e iki tekne arasında paylaştırma ve Gazebo'da iki tekne.

**6a yazıldı (11 Kasım): `usv_core/mission.py`.** Plan, kuyruk, acil istekte askıya alma ve
geri dönme (görev `start` ile yeniden başlar, hafıza ortak), yetenek denetimi (profil),
görev ve koşu zaman aşımı, HOME (şimdilik boşta durma; sürüş 6c'de), olay listesi. Görevler
fabrikalarla yaratılır, isteğin parametreleri göreve ulaşır.

**6b yazıldı (11 Kasım): bağlantı.** Navigatör açılışta `roboboat.mission`'a bakar:
`readiness` → dondurulmuş 2026 koşusu, `roboboat_2027` → görev yöneticisi (plan
`roboboat.plan`: task1, marina, task5, task2). Parkur dosyasında `mission:=roboboat_2027`.
Navigatör `task_request`'i dinler; tekne profilinin yetenekleri launch'tan gelir (Azure'a
su görevi reddedilir). Sıradaki görevler `NavStatus.task_queue` → telemetri `kq` → GCS
panelinde "SIRADAKİ". Henüz yazılmamış görevler (task1, marina, task5, task2, task4,
rally, hold) "not written yet" deyip atlanır.

**6c yazıldı (11 Kasım): `usv_core/mission_tasks.py`.** `rally` (lat, lon; isteğe bağlı
`hold_s`): noktaya git, orada dur (Task 5 Core). `hold`: olduğun yerde dur. HOME: koşunun
başladığı yere dön, orada dur. Yerinde durma bir bant: 2 m içinde motorlar boşta, sürüklenip
4 m'yi aşınca DriveTo ile geri (tahmin, Gazebo'da ölçülecek). Parametresi yanlış `rally`
sebebiyle atlanır. Barış Gazebo'da denedi: tekne noktaya gidip yerinde durdu, salınmadı.

**6d yazıldı (11 Kasım): rapor yolu.** Görevler rapor üretir (`take_reports`); görev
yöneticisi toplar; readiness koşusunda giriş/çıkış kapısı bitince "gate entry/exit" raporu.
Navigatör `task_report` (yeni `usv_interfaces/TaskReport`) yayınlar → `telemetry_bridge` rapor
paketi (`rp`, iki hattan) → GCS onaylar (`report_ack`), kopyaları atar, teknenin günlüğüne
yazar ve bu yılın protokolünde mesajı olanları RoboCommand'a verir (2026: kapı, nesne,
teslimat, yanaşma). Onay gelmeyen rapor tekneden her 1,2 s'de yeniden gönderilir. `rally`
varınca "arrived" raporu üretir (2026'da karşılığı yok: yalnız GCS günlüğünde).

**6e, birinci yarı yazıldı (12 Kasım): Gazebo'da iki tekne.** `sim.launch.py boats:=azure,marine`.
Dünya üreteci her tekneye `usv_boat` modelinin bir kopyasını koyar (`usv_azure`, `usv_marine`;
Gazebo konuları `/usv_<tekne>/...`), ikincisi başlangıcın 6 m sağında. Her teknenin kendi köprüsü,
sim_bridge'i, algılaması, yön kaynağı, haritası, navigatörü ve telemetri köprüsü; gürültü
tohumu her teknede farklı. İki tekne aynı sahte RFD kanalında. Tek tekneli koşu da artık
`usv_azure` adlı modelle (eskiden `usv_boat`). Bilinen eksik: teknenin kamerası öbür tekneyi
görmüyor (sınıfı yok), yalnız Gazebo'da çarpışırlar.
Marine Gazebo'da siyah (Barış). Ölçüm (12 Kasım, Barış'ın dizüstü): iki tekneyle Gazebo'nun
gerçek zaman oranı %34 (tek tekneyle %68; temiz bir container'da yeniden ölçüldü); işlemci toplamda %87 boşta, darboğaz Gazebo
sunucusunun tek çekirdeği (her teknenin gövde kaldırma kuvveti gövde ağı üzerinden
hesaplanıyor). Düğümlerimiz duvar saatiyle çalıştığı için yavaş simülasyonda zamanlar
uzar; davranış denemeleri için yeterli, süre ölçümleri aynı oranda karşılaştırılmalı.

**6e, ikinci yarı yazıldı (12 Kasım): paylaştırma (`usv_core/allocation.py`, GCS kullanır).**
RoboCommand'dan gelen görevi GCS kendisi bir tekneye verir, her seferinde aynı kurallarla:
hakem bir tekne adlandırdıysa o (yapamıyorsa red); yoksa duyulan, otomatikteki ve görevi
yapabilen tekneler arasından önce boşta olan (HOME, WAIT_START), sonra sırada daha az görevi
olan, sonra küçük numara (Azure). Su gerektiren görev (task4) Marine'e. Atanamayan görev
sebebiyle GCS günlüğüne yazılır. Rozet (çok görev) sırayla, her atama yük sayılarak
paylaştırılır. 2026 protokolünde hakemden görev gelmediği için GCS'de "Hakem görevi (test)"
düğmesi: task4, toplanma (iki teknenin ortası + 20 m kuzey), task1. Bununla Adım 6 tamam.

### Adım 7. Atış sistemi (Marine)
- `usv_core/turret.py`: saf mantık.
- `turret` düğümü: Jetson sürücüsü ve sahte sürücü. Pinler, I2C ve kanallar ayar dosyasında.
- Görev yöneticisinden komut alır: nişanla, N saniye su, bir top. Taret kendi kendine hedef
  seçmez.
- `atis_sistem.py`'deki sorunlar düzeltilir:
  - step motor ayrı bir iş parçacığında döner;
  - takipte de ±100° sınırı uygulanır;
  - sıfırlama eklenir;
  - röle ancak nişan alınınca açılır;
  - aynı hedefe tekrar ateş edilmez;
  - kapanışta top düşürme kontrol edilir.
- ZED derinliği menzil ve BLDC hızı için kullanılır.
- Barış'a sorular: taretin sıfırlama için limit anahtarı var mı? `cleanup()` servoları 0'a
  çekince kalan toplar düşüyor mu?

### Adım 8. Marine'de iki ZED ölçümü (donanım, Barış)
- İki ZED2i aynı AGX Orin'de. ZED SDK birden çok kamerayı seri numarasıyla açabiliyor.
- Sınırlar: USB bant genişliği (kameralar ayrı USB 3 portlarına) ve GPU (iki derinlik + iki
  YOLO).
- Tahmin değil ölçüm: küçük bir betikle iki kamera açılır, çözünürlük ve derinlik modu
  denenerek FPS ve GPU yükü ölçülür.
- Sonuca göre: taret kamerasında derinlik yalnız hedef kutusunda hesaplanır ya da çözünürlük
  düşürülür. Kaldırmazsa taret kamerasında derinlik hiç alınmaz ya da yerine daha basit bir
  USB kamera (Logitech Brio gibi) takılır (Barış).
- Adım 7'den önce ya da onunla birlikte yapılır.

### Adım 9. Task 1: Safe Passage (Gazebo)
Her adımın başında önce analiz ve plan, Barış kapsamı seçer; kod sonra.
- Başlarken: simülasyonun varsayılanı `competition` başlangıcı ve `roboboat_2027` görevi olur
  (readiness yalnız `mission:=readiness` ile; Adım 6'da böyle kararlaştırıldı).
- Parkur modeli 2027: haritadaki beacon'lardan ENTRY (yanıp sönen mavi) ve EXIT (sabit mavi),
  kırmızı, yeşil, sönük şamandıralar; görev alanının yeri (`competition` başlangıcının önünde).
- Yan kısıtlı yol: yeşil teknenin soluna (port), kırmızı sağına (starboard), sönük = engel;
  hiçbir şeye değmeden ENTRY'den EXIT'e. Elimizde: `roboboat_avoid` (engelden kaçan yol), A*,
  `PathFollower`.
- ENTRY etrafında saat yönünde, EXIT etrafında saat yönünün tersine tam tur (`Circle` var).
- Raporlar: ENTRY, EXIT ve şamandıraların türü ve rengi (`TaskReport` → GCS → RoboCommand).
- Seviyeler: Core (`layout: primer` ve rastgele `layout:=<n>`), sonra Advanced (EXIT yer
  değiştirir → yeniden planla), Disruptive (renkler değişir; tek kareye bakıp karar verme).
- Ölçüm: birkaç düzen ve seed, temas 0, her şamandıra doğru tarafta, süre.

### Adım 10. Task 3 marina, Core (Gazebo)
- Marinayı bul; bölme göstergelerinden boş (yeşil) bölmeyi seç.
- Dar bölmeye yanaşma: bölmenin ekseninde yavaş giriş, içeride durma (bölme 1,5 m, tekne
  0,78 m: her yanda ~0,35 m pay), sonra geri çıkış.
- Rapor: yanaşma (hangi bölme).
- Advanced (yangın: kırmızı pencereye su) ve Disruptive (ışık dizisi, top) Adım 7'den sonra.

### Adım 11. Task 5: Dynamic Response, Core (Gazebo)
- Sürüş tarafı yazılı (`rally`, yerinde durma; 6c). Kalan: Task 1 ya da marina sürerken gelen
  toplanma isteğinde araya girip sonra devam etmenin uçtan uca denenmesi (GCS "Judges' task
  (test)" ile), varış raporu, yerinde durmanın Gazebo'da ölçülmesi (2 m / 4 m bandı tahmin).
- Advanced/Disruptive (iki tekne birlikte rozet) 2027 protokolü ve rozet seviyeleri belli olunca.

### Adım 12. Task 2: Emergency Response Sprint, Core (Gazebo)
- Yeşil numaralı RoboBuoy'dan başla, doğru renkteki Polyform şamandıranın etrafında tur, çıkış
  şamandırasına süratle git. "Doğru renk" nereden geliyor belli değil (el kitabı): şimdilik
  parametre.
- Hızlı sürüş ayarları (sürat puanı olası). Pinger seviyeleri hidrofon kararından sonra.

### Adım 13. Tam koşu
- Plan `task1 → marina → task5 → task2`, sonra eve dönüş; tek tekne, sonra iki tekne (planı GCS
  paylaştırır). Koşu raporu: görev başına süre, temas, raporlar.

### Sıra (Barış'la, 11 ve 14 Kasım)
Adım 9 (Task 1) → Adım 10 (marina Core) → Adım 11 (Task 5 Core) → Adım 12 (Task 2 Core) →
Adım 13 (tam koşu). Üç rozet de Task 1, 3 ve 5'i istiyor; bu sırada hiçbiri su ya da top
gerektirmiyor. Faz 7 (gerçek tekne) Barış'ın sınavlarından sonra, video tarihine göre araya
girer. Adım 7 (atış sistemi) ve Adım 8 (iki ZED) donanım hazır oldukça; su/top isteyen
seviyeler onlardan sonra; Task 2'nin pinger seviyeleri hidrofon kararından sonra. El kitabı
çıkınca bu plan §9'daki sorularla yeniden gözden geçirilir.

### Bekleyenler
- Gerçek tekne (Faz 7; Proof of Readiness videosu Ocak'ta; yedek eski kod).
- Gazebo'da taret; ikinci kameranın haritaya katılması (taret açısı + tekne yönü).
- Teknenin kamerasının öbür tekneyi görmesi (iki tekne birbirini engel olarak görmeli).
- 3D LiDAR (Azure), hidrofon.
- 2027 el kitabı ve RoboCommand 2027.
- 2027 veri seti ve YOLO modeli (Barış ve takım).

### Mission Readiness (takım işi, kod dışı)
- 6 sayfalık TDR, site, "Engineering in Action", poster, mülakat, sistem değerlendirmesi.
- `ARCHITECTURE.md`, `sistem-raporu.md` ve test/simülasyon sonuçlarımız TDR'nin "kanıta dayalı"
  kısmı için hazır malzeme.

---

## 9. El kitabı çıkınca bakılacak sorular

1. Task 1: ENTRY ve EXIT şamandıraları nasıl tanınıyor (mavi ışık mı)? Siyah = sönük ışık mı?
   Raporlanacak bilgiler neler?
2. Task 2: "doğru renkteki" polyform şamandıra nereden öğreniliyor? Sürat nasıl puanlanıyor?
   Pinger frekansı ve yayın biçimi?
3. Task 3: bölme ölçüleri; "yanaşmış" sayılmanın şartı; su hedefi ne kadar su istiyor;
   raketbol nasıl teslim ediliyor?
4. Task 4: su teslimatı tam olarak ne (püskürtme mi, bırakma mı)? Şekil teknenin neresinde?
5. RoboCommand 2027: hakemden tekneye mesajlar nasıl geliyor (kıyı bilgisayarı üzerinden mi)?
   Otonomken kıyıdan bilgi alma yasağı nasıl değişiyor?
6. Mission Badge seviyeleri ve puanları; görev puanları.
7. Proof of Readiness: "iki kapı seti" tam olarak ne; video süresi ve şartları; son teslim
   tarihi.
8. İki tekne kuralları: giriş, süre, IVC 2027'de nasıl puanlanıyor?
9. Parkurda GPS düzeltmesi: 2026'da internet/LTE düzeltmesi yasaktı, ama "operasyon
   çadırındaki ekipman" serbestti. **Çadırda kendi RTK baz istasyonumuz** serbest mi?
   (Serbestse GPS kayması sorununu büyük ölçüde çözer.)

---

## 10. Ayrıntılı yorumlar (Barış'ın soruları, 6 Kasım)

Hepsi el kitabı çıkmadan önceki **yorum ve tahmin**. Dayanaklar: 2027 Primer'ın metni ve
çizimleri, 2026 el kitabındaki benzer görevler (2026 Task 3 hız parkuru, Task 4 ikmal, Task 5
marina, Task 6 ses), RobotX belgeleri ve RobotX'in geçmiş yıllardaki görevleri.

### Task 3: alttaki 1, 2, 3 numaralı kutular ne? Su ve top nerede atılıyor?

- Çizimde iki şey var:
  - **Sağdaki dikey iskele:** marinanın üstten görünüşü; üç bölme, her birinde renk
    göstergesi (yeşil/kırmızı nokta) ve beyaz bir çubuk. Çubuk büyük olasılıkla bölmedeki
    "bina", yani Docking Bay Structure.
  - **Alttaki yatay sıra:** 1, 2, 3 numaralı, her birinde **iki kare (iki pencere)** ve altında
    renkli nokta olan üç yapı.
- RobotX yanaşma yapısı tam olarak bu: **iki pencereli bir bina** (pencereler LED panel),
  tabanında bölmeye bakan bir **renk göstergesi**, önünde **su hedefi** (21×28 cm ağızlı bidon,
  akış sensörüyle). Alttaki kutular bu binaların **önden görünüşü** gibi duruyor.
- Ama renkler tutmuyor: sağda yukarıdan aşağı yeşil-kırmızı-kırmızı, altta 1-2-3 sırasıyla
  kırmızı-yeşil-kırmızı. Hiçbir bakış yönü bunu eşlemiyor. İki okuma var:
  1. Çizim yalnız örnek, renkler özensiz; alttaki sıra aynı üç binanın önden görünüşü.
  2. Alttaki sıra **başka bir iskelede** üç numaralı hedef. Görev metnindeki (c) "kaynakları
     başka bir yere teslim et" kısmı bunlar olabilir.
- **En olası yerleşim:**
  - "Yangını söndürme" (Advanced) **yanaştığın bölmenin içinde**: bölmedeki binanın
    penceresi kırmızı yanıyor ve tekne, burnu binaya dönük, yakından (1–2 m) önündeki su
    hedefine püskürtüyor. Metin de "yanaştıktan sonra yangını söndür" diyor.
  - Disruptive'deki su ya da top teslimatı ise **başka bir yerde**: "distractor" bir teknede.
    Hangi tekne olduğunu pencerenin ikinci rengi söylüyor. Bu tekneler alttaki numaralı yapılar
    ya da marinadaki numaralı tekneler olabilir.
- **Donanım sonucu:** su topu tekneden **ileriye** nişan almalı. Hedef 1–2 m'de ve teknenin
  yüksekliğine yakın (bidonun ağzı). 2026'da şartı "hedefe en az 3 saniye kesintisiz ve görünür
  su" idi. Top da büyük olasılıkla ya bir deliğe ya da teknenin içine atılıyor (2026: topu
  teknenin içine bırakmak da kabuldü).

### Task 1: renkler değişirse ne yapmalıyız? Başlangıcı nasıl buluruz? Yolu nasıl çizeriz?

**Kural büyük olasılıkla geçtiğin anda geçerli.** Bir şamandıranın hangi tarafından geçtiğine,
yanından geçtiğin anda yanan rengine göre bakılır.
- Önünde ve solunda kalacak bir yeşil, sen geçmeden **kırmızıya dönerse** artık onu **sağında**
  bırakman gerekir. Geri dönmek gerekmez: **o şamandıranın öbür tarafından dolanacak şekilde
  yolu yeniden çizersin**.
- Zaten geçtiğin bir şamandıranın rengi değişirse büyük olasılıkla önemi yoktur. (El kitabında
  bakılacak.)
- Yani istenen davranış "**renkleri canlı izleyip yolu sürekli yeniden planlamak**". Bizim
  planlayıcı her 0,25 s'de yeniden planlıyor; eksik olan, renk değişince haritadaki rengin de
  hemen değişmesi. Şimdiki harita rengi son 15 görülmenin çoğunluğuyla belirliyor; bu, ışığın
  değiştiği yerde yavaş kalır.

**Advanced'de "EXIT değişir":** büyük olasılıkla çıkış olarak işaretli şamandıra (çizimde mavi)
başka bir şamandıraya geçiyor. Tekne yeni çıkışı görüp hedefini değiştirmeli.

**Başlangıç (ENTRY) nasıl bulunur?**
- "Doğru ENTRY ve EXIT noktalarını bul ve raporla" deniyor; yani adaylar arasında
  seçmek gerekiyor.
- Çizimde giriş ve çıkış şamandıralarının ışığı **mavi**. Büyük olasılıkla mavi =
  giriş/çıkış. Belki yanıp sönme deseni de bir şey söylüyordur.
- **Strateji:** alana girmeden önce kenarından bakıp bütün ışıklı şamandıraları renkleriyle
  haritaya almak (bizim kalıcı harita bunu zaten yapıyor; ihtiyaç olursa Search ile çevresini
  taramak). Sonra:
  - **ENTRY** = bize yakın taraftaki mavi,
  - **EXIT** = alanın öbür ucundaki mavi.

**Yol nasıl çizilir?**
- Kapı yok, her şamandıra tek başına bir yan kısıtı koyuyor: **yeşil soldan, kırmızı sağdan**
  kalacak.
- Bizim A* planlayıcı bunu **sanal duvarlarla** yapabilir. Kanal kapılarında kullandığımız
  duvarların aynısı:
  - Her yeşilden gidiş yönünün **soluna**, her kırmızıdan **sağına** doğru birkaç metrelik bir
    duvar çizilir. Böylece A* yeşili ancak sağından, kırmızıyı ancak solundan geçebilir.
  - Siyahlar (sönük) sıradan engel.
  - "Gidiş yönü" ENTRY'den EXIT'e doğru.
- Ortaya çıkan yol, kırmızı-yeşil çiftlerini doğal olarak "kapı" gibi kullanır. Renk değişince
  duvar öbür tarafa geçer, yol da kendiliğinden yeniden çizilir.

### Task 2: akustik pinger nedir, nerede olur? Görev tam olarak ne?

- **Pinger:** su altına asılan, belli aralıklarla kısa ses darbeleri (genelde 25–40 kHz,
  insan kulağının üstünde) yayan küçük bir cihaz. Ses suyun içinde yayılır, havadan duyulmaz.
- **Nerede:** büyük olasılıkla **numaralı RoboBuoy'ların (1, 2, 3) altında**, bir iple asılı.
  Lacivert yuvarlak şamandıra değil. Gerekçe: Advanced'de çıkış "pingeri çalışan şamandıranın
  yanından", Disruptive'de başlangıç ve bitiş "yalnız pingerle bulunur"; aday başlangıç/bitiş
  şamandıraları da çizimdeki 1, 2, 3. Yalnız biri ya da ikisi açık olur.
- **Çizimdeki görev akışı:** tekne 3 numaralının (tepesi yeşil) yanından başlıyor; oklar
  yukarı çıkıp **lacivert şamandıranın etrafından** dolanıyor ve aşağı iniyor. Yani lacivert
  bu örnekte "doğru renkteki hedef". Kırmızı, sarı, siyah, yeşil, beyaz diğer renk
  seçenekleri ya da engeller.
- **Seviyeler:**
  - Core: yeşil başlangıç → doğru renkteki hedefin etrafında tur → alandan çık.
  - Advanced: aynısı, ama çıkış pingeri çalışan numaralı şamandıranın yanından.
  - Disruptive: başlangıç da bitiş de yalnız pingerle bulunuyor; şamandıralar gözle ayırt
    edilemiyor.
- **"Doğru renk" nereden öğreniliyor?** Yazılmamış. Disruptive'de "atanan hedef" deniyor;
  büyük olasılıkla RoboCommand söylüyor. 2026'da tur yönünü yanındaki renk göstergesi
  söylüyordu.
- **Teknede ne gerekir?** Su altında en az 2, tercihen 3–4 **hidrofon** (su altı mikrofonu),
  hızlı bir örnekleyici (≥200 kHz) ve sinyal işleme. Sesin hidrofonlara varış zamanı farkından
  (TDOA) **yön** bulunur. Tekne hareket ettikçe birkaç yön kesiştirilerek **konum** bulunur.
  Ciddi bir donanım ve yazılım işi; RobotX takımları bunu yapıyor.

### Task 5: kırmızı daire toplanma alanı mı? Çizimde tekne ondan kaçıyor gibi.

- Haklısın, çizimde tekne kırmızı kesikli dairenin **etrafından dolanıp uzaklaşıyor**.
  Kırmızı ok da dairenin merkezinden dışarı çıkıyor.
- Metin ise Core için "isteneni bırakıp **istenen GPS noktasında toplan**" diyor. Çizim
  metinle tam uyuşmuyor. İki okuma var:
  1. Çizim genel bir "beklenmedik görev değişikliği" resmi: tekne yolunu bırakıp başka yöne
     gidiyor. Daire yalnızca bir bölge örneği.
  2. Kırmızı daire **yeni ilan edilmiş, kaçınılacak bir tehlike bölgesi**, ok da bölgenin
     **hareket yönü** (örneğin sürüklenen bir tehlike). Tekne de o anki görevini bu bölgeden
     kaçınarak sürdürüyor. Bu, ileri seviyelerdeki bir senaryo olabilir.
- Bizim için ikisi de ucuz: GPS noktasına gitmek DriveTo, bir bölgeden kaçınmak da A*'a bir
  daire engel eklemek. Asıl iş **RoboCommand'dan mesaj almak** ve **görevi kesip yenisine
  geçmek**.

### Task 4: teslimat teknelerinin yerleri verilecek mi, yoksa kendimiz mi bulacağız?

- Metin: RoboCommand "bir ya da birkaç tekneye su götür" mesajı gönderiyor; tekneler
  yelkenlerindeki **şekilden** (artı, daire, kare, üçgen) tanınıyor. Konum verileceği
  yazılmıyor.
- 2026'da ikmal tekneleri "parkurun her yerine demirlenmişti" ve takımlar onları **kendileri
  bulup yerlerini raporluyordu**. 2027'de de büyük olasılıkla mesaj "**üçgen tekneye su**"
  diyecek, yeri söylemeyecek.
- **En iyi strateji:** tekneler koşu boyunca zaten etrafta. Kalıcı haritamız, diğer görevleri
  yaparken gördüğü her coroplast tekneyi **şekliyle birlikte** haritaya almalı. Mesaj
  geldiğinde tekne "üçgen nerede?" diye haritaya bakar ve doğrudan gider. Haritada yoksa
  arar. Bu, açılıştan itibaren kalıcı harita kararımızın tam karşılığı.
- **Teslimatın kendisi** (2026'ya göre): teknenin yanındaki şekle **en az 3 saniye
  kesintisiz, görünür su** püskürtmek. Su topu yana ya da ileri nişan alabilmeli; tekne de
  hedefin yanında **yerinde durabilmeli** (station keeping, Primer'ın listesinde var).

### Mission Badge (rozet) nedir? Neden üçünde de aynı üç görev var? Task 5 ile ilişkisi ne?

- **Görevler** tek tek seviyelerine göre puanlanıyor. **Rozet** ise **tek bir koşuda** belli
  bir görev bileşimini belli seviyelerde tamamlayınca verilen ek bir başarı. Sıralamaya katkı
  yapıyor.
- Üç rozetin üçü de aynı üç görevi istiyor: **Safe Passage (Task 1) + Navigate the Marina
  (Task 3) + Dynamic Response (Task 5)**. Ama her biri için "**önceden belirlenmiş seviye**"
  diyor ve seviyeleri henüz vermedi. Büyük olasılıkla fark seviyelerde. Örnek (tahmin):
  - **Assessment and Discovery** (keşif: görme, kaçınma): Task 1 yüksek seviyede, marina ve
    Task 5 temel seviyede.
  - **Automated Logistics Support** (lojistik: yanaşma, teslimat): marina yüksek seviyede
    (yangın, teslimat).
  - **Dynamic Tasking** (dinamik görev: RoboCommand'a cevap): Task 5 yüksek seviyede.
- **Task 5 ile bağlantı:** Task 5 Advanced'de RoboCommand koşunun başında "**şu rozeti yap**"
  diyor. İki tekne o rozetin görev bileşimini **birlikte** tamamlamak zorunda; örneğin biri
  Safe Passage'ı, öbürü marinayı yapar. Disruptive'de bu istek tekneler başka işlere
  başladıktan sonra geliyor: planı koşu ortasında değiştirmek gerekiyor.
- Yani rozetler hem bir puan kalemi hem de Task 5'in ileri seviyelerinde **hakemin verdiği
  görevin kendisi**. Görev yöneticimiz "rozet X = şu görevler, şu seviyelerde" bilgisini
  tutmalı ve iki tekne arasında paylaştırabilmeli.

### Task 1 şamandıraları: hepsi aynı mı, ışık mı renk göstergesi mi? (çizimler büyütülerek)

- **Gövde hepsinde aynı:** beyaz RoboBuoy piramidi, yüzünde siyah sembol (parkur yerleşiminde
  elmas; Task 1 çiziminde daire çizilmiş, büyük olasılıkla özensizlik). Yani "farklı tip
  şamandıra" yok; **fark tepedeki ışıkta**.
- **Tepede yeşil, kırmızı ve mavi olanların etrafında ışın çizgileri var (yanıyor); "siyah"
  olanlarda yok (sönük).** Metin de "RED, GREEN, and BLACK (OFF)" diyor: siyah = **ışığı
  kapalı**.
- **Light Beacon (ışıklı işaretçi) oldukları neredeyse kesin:**
  1. **Mavi** var; Color Indicator mekanik kanatlı, yalnız kırmızı/yeşil gösterebiliyor.
  2. **"Sönük" (OFF)** durumu var; renk göstergesinin sönük hali yok, ışığın var.
  3. RobotX'te Light Beacon'lar Task 1 RoboBuoy'larında kullanılıyor; Color Indicator ise
     başka görevlerde ve yanaşma yapısının tabanında.
  4. Renkler **koşu sırasında değişiyor** (Advanced/Disruptive): LED işaretçi Wi-Fi'den anında
     değiştirilebiliyor; mekanik göstergede servo dönüyor, daha yavaş.
- **İki mavi farklı çizilmiş:** girişteki (ENTRY) mavinin ışınları var, çıkıştakinin (EXIT)
  yok. İki çizimde de böyle. Okumalar: çizim özensizliği; ya da ikisi farklı davranıyor
  (biri yanıp sönüyor, biri sabit; ya da çıkış henüz "açılmamış"). Advanced'de "çıkış
  değişiyor" dendiği için çıkışın sonradan yanan/değişen mavi olması akla yatkın. **El kitabında
  bakılacak.**
- **Algılama için sonuçları:**
  - YOLO'ya **tek sınıf "RoboBuoy"** (sembol türüyle: elmas/daire/harf) öğretip, **ışık
    durumunu ayrı** bulmak daha sağlam: kutunun üst kısmındaki işaretçi bölgesinin rengi ve
    parlaklığı → kırmızı / yeşil / mavi / sönük. Renk değişince model yeniden eğitilmez.
  - Işık **yanıp sönebilir** (işaretçinin yazılımı yanıp sönme hızını ayarlayabiliyor). Tek
    karede "sönük" görünen bir yeşil aslında yanıp sönüyor olabilir. Karar birkaç karelik
    (≈1–2 s) pencerede verilmeli: "bu sürede hiç yandı mı, hangi renkte?"
  - Gün ışığında LED rengi soluk görünür; işaretçinin üstündeki siyah şapka bunun için.
    Kamera pozlaması ve renk eşikleri gerçek ışıkla (ya da RobotX'in yayımladığı malzemeyle
    yapılmış bir örnekle) denenmeli.
