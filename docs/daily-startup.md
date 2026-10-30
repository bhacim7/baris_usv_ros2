# Günlük Çalışma Rehberi

Bilgisayarı açtıktan sonra ROS 2 ortamına girmek, çalışmak ve günü kapatmak için gereken her şey.
Komutları yukarıdan aşağı sırayla çalıştır. Terminale yapıştırma kısayolu: `Ctrl + Shift + V`.

---

## 0. Önce bil: hangi komut nerede çalışır

| Satır başı | Neredesin | Burada ne yapılır |
|---|---|---|
| `baris@...` | Ana makine (Ubuntu) | `docker ...`, `git ...`, dosya oluşturma ve düzenleme |
| `ubuntu@...:/ws$` | Container'ın içi | `ros2 ...`, `colcon build`, `rqt_graph`, Gazebo |

- `ros2: command not found` görürsen ana makinedesin; container'a girmen gerekiyor.
- `docker: command not found` görürsen container'dasın; ana makine terminaline geç.

| Yer | Ne var |
|---|---|
| `~/usv_ros2` | ROS 2 projesi (git deposu, GitHub: `bhacim7/baris_usv_ros2`) |
| `~/usv_ros2/ws/src` | Kendi paketlerin. Container içinde `/ws/src` olarak görünür |
| `~/usv_ros2/docker/Dockerfile` | İmajın tarifi |
| `~/usv_ros2/compose.yaml` | Container'ın nasıl başlatılacağı |
| `~/usv_ros2/docs/ARCHITECTURE.md` | Onaylanmış mimari ve yol haritası (İngilizce) |
| `~/usv_ros2/docs/open-items.md` | Senden beklenen dosyalar, yapılacak kontroller |
| `~/usv_legacy` | Eski multiprocess kod. Yalnızca başvuru için; içinde değişiklik yapma |

| Paket (`ws/src/`) | Ne |
|---|---|
| `usv_interfaces` | Özel mesaj ve service tanımları |
| `usv_core` | ROS'suz algoritma kütüphanesi (taşınan eski kod) ve testleri |
| `usv_sim` | Simülasyon teknesi (`usv_boat`) ve dünya (`nbpark`) |
| `usv_learn` | Öğrenme paketi; sistemin parçası değil |

---

## 1. Açılış

GRUB menüsünde ok tuşlarıyla **Ubuntu**'yu seç ve Enter'a bas. Varsayılan Windows olduğu için seçmezsen Windows açılır.

---

## 2. Container'ı başlat ve içine gir

Ana makinede bir terminal aç (`Ctrl + Alt + T`):

```bash
cd ~/usv_ros2
```

```bash
xhost +local:docker
```
Beklenen: `non-network local connections being added to access control list`
Gazebo, RViz ve `rqt_graph` pencerelerinin açılabilmesi için. Her oturum açılışında bir kez gerekir.

```bash
docker compose up -d
```
Beklenen: `Container usv  Started` ya da `Running`.

```bash
docker compose ps
```
Beklenen: `usv` satırında `Up` yazıyor.

```bash
docker compose exec usv bash
```
Beklenen: satır başı `ubuntu@...:/ws$` olur. Artık container'ın içindesin.

İçeride her şeyin yerinde olduğunu doğrula:

```bash
ros2 pkg list | grep usv
```
Beklenen: `usv_interfaces` ve `usv_learn`. `usv_core` ve `usv_sim` henüz `colcon` ile derlenmedi; testleri ve simülasyonu doğrudan klasörlerinden çalıştırıyoruz (3. bölüm).

### Ek terminal gerektiğinde

Yeni sekme aç (`Ctrl + Shift + T`) ve yalnızca şunu çalıştır:

```bash
cd ~/usv_ros2 && docker compose exec usv bash
```

Yeni container terminallerinde ROS 2, VRX ve kendi paketlerin kendiliğinden yüklenir; `source` yazmana gerek yoktur.

---

## 3. Sık yapılan işler (container içinde)

### Derleme

```bash
cd /ws && colcon build --symlink-install && source install/setup.bash
```
Beklenen: `Summary: N packages finished`

Ne zaman gerekir:
- Yeni paket oluşturduğunda
- `setup.py`, `package.xml` ya da `CMakeLists.txt` değiştiğinde
- Yeni dosya eklediğinde (launch, config, yeni node dosyası)
- Mesaj (`.msg`) ya da service (`.srv`) tanımı değiştiğinde

Ne zaman gerekmez: yalnızca mevcut bir Python dosyasının ya da YAML dosyasının içini değiştirdiğinde (`--symlink-install` sayesinde).

Derlemeden sonra **diğer açık terminallerde** de `source /ws/install/setup.bash` çalıştır ya da o terminalleri kapatıp yeniden aç.

### Node ve launch çalıştırma

```bash
ros2 run usv_learn fake_gps
```

```bash
ros2 launch usv_learn learn.launch.py
```
Durdurmak için: `Ctrl + C`

### `usv_core` testleri

Taşınan eski kodun eski kodla aynı sonucu verdiğini kontrol eder. Kodda bir şey değiştirdikten sonra, commit'ten **önce** çalıştır:

```bash
cd /ws/src/usv_core && python3 -m pytest test/test_navigation.py test/test_planner.py test/test_object_memory.py test/test_mixer.py
```
Beklenen: `487 passed` (test eklendikçe sayı artar). Kırmızı `FAILED` varsa commit etme.

### Kendi simülasyonumuz (`usv_sim`)

Gazebo'ya kendi model klasörümüzü göster (her yeni terminalde bir kez):

```bash
export GZ_SIM_RESOURCE_PATH=/ws/src/usv_sim/models:$GZ_SIM_RESOURCE_PATH
```

```bash
gz sim -v 3 /ws/src/usv_sim/worlds/nbpark.sdf
```
- Dünya **duraklatılmış** açılır (sağ altta `0.00 %`). Sol alttaki **▶**'e bas.
- Entity tree'de `usv_boat`'a sağ tık → **Follow**: kamera tekneyi takip eder.
- Gazebo'yu `Ctrl + C` ile kapatıp açmak tekneyi başlangıç noktasına döndürür.

İkinci bir container terminalinde Gazebo'nun kendi komutları (`gz`, `ros2` değil):

| Komut | Ne yapar |
|---|---|
| `gz topic -l \| grep usv_boat` | Teknenin Gazebo topic'leri |
| `gz model -m usv_boat -p` | Teknenin anlık konumu ve açısı |

Konumu 5 kez, 1 saniye arayla ölç:

```bash
for i in 1 2 3 4 5; do gz model -m usv_boat -p 2>/dev/null | grep -A1 "Pose" | tail -n 1; sleep 1; done
```

Dört iticiye aynı itki (N; ileri pozitif):

```bash
for t in front_left front_right rear_left rear_right; do gz topic -t /usv_boat/thrusters/$t/thrust -m gz.msgs.Double -p 'data: 10.0'; done
```

Ön iticilerin açısı (rad; + = yukarıdan bakınca saat yönünün tersi, tekne sola döner; sınır ±0,785):

```bash
for t in front_left front_right; do gz topic -t /usv_boat/thrusters/$t/pos -m gz.msgs.Double -p 'data: 0.5'; done
```

Hepsini durdur:

```bash
for t in front_left front_right rear_left rear_right; do gz topic -t /usv_boat/thrusters/$t/thrust -m gz.msgs.Double -p 'data: 0.0'; done
```

### Parkurlu simülasyon (ROS 2 ile)

Tek komut (dünyayı parkur dosyasından kendisi üretir, her şeyi açar; Ctrl+C hepsini kapatır):

```bash
ros2 launch usv_sim sim.launch.py
```

Seçenekler: `paused:=true` (duraklatılmış başlar, ▶'e bas), `gui:=false` (pencere yok, daha hızlı), `course:=<ad>` (başka parkur).

Elle, tek tek açmak istersen (bir node'u ayrıca izlemek için): parkur dosyası değiştiyse önce dünyayı yeniden üret:

```bash
python3 /ws/src/usv_sim/scripts/make_course_world.py /ws/src/usv_sim/courses/golcuk_2026_10_14.yaml /ws/src/usv_sim/worlds/golcuk_2026_10_14.sdf
```

Her biri ayrı terminalde, bu sırayla (Gazebo'da ▶'e basmayı unutma):

| # | Komut | Ne |
|---|---|---|
| 1 | `export GZ_SIM_RESOURCE_PATH=/ws/src/usv_sim/models:$GZ_SIM_RESOURCE_PATH && gz sim -v 3 /ws/src/usv_sim/worlds/golcuk_2026_10_14.sdf` | Gazebo, Gölcük parkuru |
| 2 | `ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=/ws/src/usv_sim/config/ros_gz_bridge.yaml` | ROS ↔ Gazebo köprüsü |
| 3 | `ros2 run usv_sim sim_bridge --ros-args --params-file /ws/src/usv_sim/config/sim_bridge.yaml` | Cube tarafı: itici, dümen, röle, GPS, hız, FC pruvası |
| 4 | `ros2 run usv_sim fake_perception --ros-args --params-file /ws/src/usv_sim/config/fake_perception.yaml` | ZED tarafı: ZED pruvası, yalpa/baş-kıç, FPS |
| 5 | `ros2 run usv_nodes heading_source --ros-args --params-file /ws/src/usv_nodes/config/heading_source.yaml` | `/usv/heading` |

Elle sürmek: `ros2 topic pub -r 20 /usv/thrusters/pwm usv_interfaces/msg/ThrusterPwm "{rear_left: 1680, rear_right: 1680, front_left: 1680, front_right: 1680, steer: 1500}"` (Ctrl+C ile bırakınca 0,5 sn sonra durur).

### VRX'in hazır simülasyonu

```bash
ros2 launch vrx_gz competition.launch.py world:=sydney_regatta
```
İlk açılışta modeller indiği için pencere birkaç dakika boş kalabilir; "yanıt vermiyor" uyarısında **Wait** de.

### İnceleme araçları

| Komut | Ne gösterir |
|---|---|
| `ros2 node list` | Çalışan node'lar |
| `ros2 topic list` | Var olan topic'ler |
| `ros2 topic echo /usv/fix --once` | Bir topic'ten tek mesaj |
| `ros2 topic hz /usv/fix` | Topic'in gerçek yayın hızı |
| `ros2 topic info /usv/fix` | Mesaj tipi, yayıncı ve dinleyici sayısı |
| `ros2 param list /distance_monitor` | Bir node'un parametreleri |
| `ros2 param get /distance_monitor arrival_radius_m` | Tek parametrenin değeri |
| `ros2 service list` | Var olan service'ler |
| `rqt_graph` | Node ve topic'lerin canlı şeması |

### Kayıt ve geri oynatma (rosbag)

```bash
mkdir -p /ws/bags && cd /ws/bags
```

```bash
ros2 bag record -o run1 --topics /usv/fix /usv/target_distance
```
`Ctrl + C` ile durdur. Aynı adla ikinci kez kayıt alınamaz; her seferinde yeni ad ver (`run2`, `run3`...).

```bash
ros2 bag info run1
```

```bash
ros2 bag play run1 --topics /usv/fix
```

Kayıtlar ana makinede `~/usv_ros2/ws/bags` altında durur ve git'e girmez.

---

## 4. Değişiklikleri kaydet (ana makinede)

```bash
cd ~/usv_ros2 && git status
```
Neyin değiştiğini gösterir.

```bash
git add .
```

```bash
git commit -m "Short English description of the change"
```

```bash
git push
```
Beklenen: `To github.com:bhacim7/baris_usv_ros2.git` ile biten satırlar.

---

## 5. İmajı yeniden derlemek gerektiğinde

`docker/Dockerfile` ya da `compose.yaml` değiştiğinde, ana makinede:

```bash
cd ~/usv_ros2 && docker compose build
```

```bash
docker compose up -d
```
Beklenen: `Container usv  Recreated` ve `Started`.

Container içinde `apt install` ile kurduğun paketler kalıcı **değildir**; container yeniden oluşturulunca gider. Kalıcı olması gereken her paket Dockerfile'a eklenir.

---

### Image kayıt noktası

Çalışan bir image'e tarihli ikinci bir ad ver (ek disk kaplamaz):

```bash
docker tag usv-ros2:dev usv-ros2:YYYY-MM-DD-aciklama
```

```bash
docker images usv-ros2
```

Yeni bir derleme ortamı bozarsa eskisine dön:

```bash
cd ~/usv_ros2 && docker tag usv-ros2:YYYY-MM-DD-aciklama usv-ros2:dev && docker compose up -d
```

Kayıtlı noktalar:

| Etiket | Ne var |
|---|---|
| `usv-ros2:2026-10-22-phase3` | ROS 2 Jazzy, Gazebo Harmonic, VRX. Faz 3 sonu |

Kodun kayıt noktası image değil, git commit'idir (4. bölüm).

---

## 6. Kodu düzenleme (VS Code)

```bash
code ~/usv_ros2
```

- Dosyaları VS Code'da düzenle ve `Ctrl + S` ile kaydet. Container aynı dosyaları `/ws` altında anında görür.
- Komutları (derleme, çalıştırma) yine container terminalinde çalıştır. VS Code'un kendi terminali (`Ctrl + J`) ana makinede açılır; oradan da `docker compose exec usv bash` ile container'a girebilirsin.
- VS Code `rclpy` gibi container'daki kütüphaneleri tanımadığı için bazı satırları sarı çizebilir; kod yine çalışır.
- VS Code'da yapıştırma `Ctrl + V`'dir (terminaldeki `Ctrl + Shift + V` değil).
- Klasörüyle birlikte yeni dosya: Explorer'da klasöre sağ tık → **New File...** → `msg/Buoy.msg` gibi yolu yaz; klasör kendiliğinden oluşur.

---

## 7. Günün sonunda

1. Çalışan node'ları ve simülasyonu `Ctrl + C` ile durdur.
2. Container terminallerinde `exit` yaz.
3. Ana makinede:

```bash
cd ~/usv_ros2 && docker compose down
```
Beklenen: `Container usv  Removed`. Kodun, derleme çıktıların ve indirilen modeller silinmez.

Bunu yapmadan bilgisayarı kapatsan da bir şey bozulmaz; bir sonraki açılışta 2. adım her şeyi yeniden başlatır.

---

## 8. Sorun giderme

| Belirti | Sebep | Çözüm |
|---|---|---|
| `service "usv" is not running` | Container kapalı | `docker compose up -d` |
| `ros2: command not found` | Ana makinedesin | `docker compose exec usv bash` |
| `permission denied ... docker.sock` | Oturum `docker` grubunu tanımıyor | Bilgisayarı yeniden başlat; acilse o terminalde `newgrp docker` |
| Gazebo ya da `rqt_graph` penceresi açılmıyor | Ekran izni verilmemiş | Ana makinede `xhost +local:docker` |
| `Package 'usv_...' not found` | Terminal yeni derlemeyi tanımıyor | `source /ws/install/setup.bash` |
| Yeni node `ros2 run` ile bulunamıyor | `setup.py` değişti ama derlenmedi | `cd /ws && colcon build --symlink-install` |
| `Driver/library version mismatch` | Sürücü güncellendi, yeniden başlatılmadı | Bilgisayarı yeniden başlat |
| `glxinfo` NVIDIA yerine Intel ya da llvmpipe gösteriyor | Çizim yanlış kartta | Dur, çıktıyı Claude'a göster |
| Uzun blok yapıştırınca ekran karışık görünüyor | Terminalin çizim hatası | Dosyayı `cat` ya da `python3 -m py_compile` ile doğrula |
| Gazebo'da hiçbir şey hareket etmiyor, sağ altta `0.00 %` | Dünya duraklatılmış | Sol alttaki ▶ |
| Gazebo `usv_boat` modelini bulamıyor | Model klasörü tanıtılmamış | `export GZ_SIM_RESOURCE_PATH=...` satırı (3. bölüm) |
| Suyun üstünde yeşil tel çerçeveli bir kare | Güneşin (ışık kaynağının) işareti | Zararsız; tekneyi bulmak için `usv_boat` → **Follow** |

### Ekran donarsa

1. **Caps Lock**'a bas. Işık yanıyorsa sistem çalışıyor, yalnızca görüntü donmuş.
2. Güç tuşuna **bir kez kısaca** bas ve **90 saniye dokunma**. Kendiliğinden kapanırsa sistem canlıydı.
3. Kapanmazsa güç tuşunu basılı tut.
4. Açılınca şu çıktıyı Claude'a gönder:

```bash
journalctl -b -1 -n 60 --no-pager
```

13 Ekim 2026'da Intel ekran kartının PSR özelliği kapatıldı (`i915.enable_psr=0`). Hâlâ kapalı olduğunu doğrulamak için:

```bash
sudo sh -c 'cat /sys/kernel/debug/dri/*/i915_edp_psr_status 2>/dev/null' | grep "PSR mode"
```
Beklenen: `PSR mode: disabled`

---

## 9. Ara sıra kontrol

Pil şarj sınırı (%80):

```bash
cat /sys/class/power_supply/BAT0/charge_control_end_threshold
```
Beklenen: `80`

Disk durumu:

```bash
df -h /
```

Docker'ın kapladığı yer:

```bash
docker system df
```
