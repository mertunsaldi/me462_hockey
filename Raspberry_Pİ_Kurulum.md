Raspberry Pi Bilgileri Ve kurulum rehberi <br>


  hostname: raspberrypi.local <br>
  username: remor <br>
  password: remor <br>

Wireless LAN
  
  SSID: Seççavv <br>
  Password: mersin3301 <br>

Kurulum ve RaspberryPi kullanma

1) Pi'a güç verin
2) Bilgisayar ve Pi'ı aynı internet ağına bağlayın
3) Bilgisayardan terminali açın ve aşağıdaki komutu girin. (username@hostname)
  ssh remor@raspberrypi.local
4) Şu an pi'ın içindesiniz.

Eğer Pi'dan masaüstü görüntüsünü almak isterseniz
1) Pi'ın masaüstü görüntüsünü almak için terminale sudo raspi-config yazın
2) Interface optionsa girip VNC serverını enablelayın
3) Internetten VNC Viewer adlı uygulamayı indirin ve kurun
4) VNC Viewera pi'ın bağlı olduğu ipyi yazarsanız kendi bilgisayarınızda pi'ın masaüstünü görebilirsiniz
