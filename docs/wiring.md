# Bağlantı özeti

Bu pin düzeni örnek Arduino yazılımıyla aynıdır. Motor sürücünün lojik gerilim ve akım gereksinimlerini veri sayfasından kontrol et.

| İşlev | Arduino pini |
|---|---:|
| Sol motor PWM | D5 |
| Sol motor yön 1 / yön 2 | D7 / D8 |
| Sağ motor PWM | D6 |
| Sağ motor yön 1 / yön 2 | D9 / D10 |
| Sol enkoder A | D2 |
| Sağ enkoder A | D3 |

## Güç güvenliği

- Motorları Arduino'nun 5 V pininden besleme.
- Motor bataryasını motor sürücüye bağla.
- Arduino, Raspberry Pi ve motor sürücünün GND uçlarını ortakla.
- Raspberry Pi'yi regüle edilmiş 5 V kaynaktan besle.
- İlk denemeyi robotun tekerleri yerden kesilmişken yap.

## USB adlarını kurma

```bash
sudo cp udev/99-ros-slam-robot.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
sudo usermod -aG dialout "$USER"
```

Oturumu kapatıp açtıktan sonra `/dev/ttyUSB_ARDUINO` ve `/dev/ttyUSB_LIDAR` bağlantılarını kontrol et.
