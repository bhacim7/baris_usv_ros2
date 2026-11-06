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
- 2026'da yön **yalnız tekneden hakeme**ydi. 2026 kuralına göre tekne otonomken kıyıdan kontrol
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

## 8. Önerilen yol haritası

**Zaman çizelgesi:** bugün 6 Kasım 2026 → teslimler Ocak 2027 → yarışma 18 Şubat 2027.
Toplam ~3,5 ay; gerçek teknenin suda otonom kapı geçişi videosu ~2 ay içinde lazım.

### A. Hemen (Kasım)
1. **Kararlar (takım):** bir mi iki tekne mi; su topu, top fırlatıcı, hidrofon donanımları
   yapılacak mı; hangi görevler ve seviyeler hedefleniyor.
2. **Takip:** 2027 el kitabını ve RoboCommand 2027'yi (`robonation/robocommand`) bekle,
   Discord'u izle (soru listesi aşağıda).
3. **Gerçek tekne yolu (Faz 7) öne çekilmeli:** Proof of Readiness videosu için tekne Jetson
   üzerinde, gerçek sensörlerle, suda kapılardan otonom geçebilmeli. Simülasyonda bu zaten
   çalışıyor (Task 1 giriş/çıkış). Bu iş yarışmaya gitmenin **ön şartı**.

### B. Altyapı (bütün görevlerin ortak ihtiyacı)
1. **RoboCommand köprüsü:** heartbeat, raporlar, ileride gelen görev mesajları; simülasyonda
   sahte sunucu. 2026 protokolüyle başlanır, 2027 çıkınca uyarlanır.
2. **Görev yöneticisi:** sıradaki görevlerin yanında **acil görevle araya girip sonra devam
   etme** (Task 4 ve Task 5 için şart).
3. **2027 parkuru simülasyonda:** RoboBuoy + ışıklı işaretçi (renk, sönük, renk değiştirme),
   polyform şamandıralar, marina ve yanaşma yapıları, şekilli tekneler.
4. **Algılama:** yeni sınıflar, LED ışık rengi ve sönük durumu, LED panel dizisi okuma.
5. **R3.5 GPS kayması tahmini** (haritanın doğruluğu bütün görevleri ve raporları etkiliyor).
6. **İki tekneye hazırlık:** isim alanları, iki tekneli simülasyon, birbirini engel olarak görme.

### C. Görevler (rozet ortaklığı ve yapılabilirliğe göre)
1. **Task 1 Core**: Circle ve kaçınma hazır; yeni olan yan kısıtlı yol ve ışık tanıma.
2. **Task 5 Core**: RoboCommand mesajı gelince GPS noktasına git; altyapı hazır olunca hızlı.
3. **Task 3 Core**: yeşil bölmeye yanaşma.
4. **Task 2 Core**: başlangıç, hedef etrafında dönüş, sürat.
5. **Task 4 Core**: su topu donanımına bağlı.
6. Advanced seviyeler: Task 1 (değişen çıkış), Task 3 (yangın), Task 4 (2 tekne).
7. İki tekne işleri: Task 5 Advanced/Disruptive, IVC.
8. Akustik (Task 2 Advanced/Disruptive): hidrofon donanımına bağlı.

### D. Mission Readiness (takım işi, kod dışı)
6 sayfalık TDR, site, "Engineering in Action", poster, mülakat, sistem değerlendirmesi.
`ARCHITECTURE.md`, `sistem-raporu.md` ve test/simülasyon sonuçlarımız TDR'nin "kanıta dayalı"
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
