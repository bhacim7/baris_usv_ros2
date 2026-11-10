# GCS (yer istasyonu)

2026'nın `GCSv1000.py`'si, RoboBoat 2027 için iki tekneye göre düzenlendi (Adım 5b):

- Solda **IDA 1 · AZURE**, sağda **IDA 2 · MARINE** paneli; harita ortada. Her panelin kendi
  telemetrisi, görev tablosu, kontrolleri, röle ve hat durumu var; komutlar o teknenin
  numarasıyla gider.
- **Ubiquiti (UDP) ve RFD aynı anda**: her komut iki hattan gider (tekne bir kez uygular),
  telemetri iki hattan gelir, her teknenin en yeni paketi gösterilir. Üst çubukta her hattın
  son duyulma süresi (`UBNT 0.1s  RFD 0.3s`). Bağlantı kodu teknedekiyle aynı:
  `ws/src/usv_core/usv_core/gcs_link.py`.
- Teknelerin Ubiquiti adresleri ve portları tekne profillerinden okunur
  (`ws/src/usv_nodes/config/boats/*.yaml`); "Simülasyon" seçilirse 127.0.0.1.
- İHA kutusu gizli (silinmedi; başka yarışmalarda gerekebilir).
- Görseller `resources/` altında.

ROS gerekmez. Gerekenler: Python 3, PySide6, numpy, pyserial.

## Çalıştırma (ana makinede, ROS'suz)

Bir kerelik kurulum (Ubuntu'da sistem Python'una pip ile kurulmaz, sanal ortam). Sanal
ortam aracı ve Qt'nin pencere açmak için istediği kütüphane (Qt 6.5'ten beri) ayrı paketler:

```bash
sudo apt install python3.12-venv libxcb-cursor0
```

```bash
python3 -m venv ~/gcs-venv
```

```bash
~/gcs-venv/bin/pip install PySide6 numpy pyserial
```

Her seferinde:

```bash
~/gcs-venv/bin/python ~/usv_ros2/gcs/gcs.py
```

## Simülasyonla

Container'da `sim.launch.py ... link:=true` çalışırken GCS'de: Ubiquiti işaretli, "Simülasyon
(127.0.0.1)", RFD işaretsiz (sahte telsizin portu container'ın içinde; RFD yolunu
`ws/tools/gcs_cli.py` ile container'dan dene), BAĞLAN. `boat:=marine` ile açılan simülasyon
sağ panelde görünür.

## Gazebo ile GCS haritası

DOĞRULANMADI: Barış'a göre tekne iki ekranda aynı yerde değil; 11 Kasım'da yarımadadaki
binadan hiza alınarak bakılacak. 10 Kasım'daki ilk açıklama şuydu:
- Gazebo'nun kamerasında yukarı kuzey olmak zorunda değil (açılışta kamera teknenin
  baktığı yöne, batıya döner); GCS haritasında yukarı her zaman kuzey.
- VRX'in park modeli sadeleştirilmiş (ör. yarımadanın doğusundaki küçük ada yok) ve
  simülasyondaki şamandıralar GCS'nin uydu görüntüsünde yok; görüntüdeki turuncu noktalar
  gerçek kürek şeridinin şamandıraları.
