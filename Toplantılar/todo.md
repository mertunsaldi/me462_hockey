#### Not: Yeni APInin kullanılması ile iligli rehberler docs altındaki "api_testing_guide.md" ve "client_scenarios.md" altında bulunabilir.

- [ ] bir master pico yapılacak
  - [ ]   sensör girdilerini toplayıp bilgisayara yollayacak
  - [ ]   plotclocklara id number verilecek (hem kod içine hem aruco markera, böylece imageda gördüğümüz plotclocka hangi id ile komut vereceğimiz net olacak)
    
  - [ ]   her plotclock uart veya i2c veya wifi ile master picoya bağlanacak, pcye sadece master bağlanacak
    - [ ]     şu anda kodda gadgets.plotclock classı her bir plotclock, usb ile ayrıca bağlanıyormuş gibi yazıldı. Bu yüzden kendi port, baudrate vs. connection bilgilerini tutuyorlar, bunun değiştirilmesi gerek
          
- [ ]   dışarıdan senaryo yüklenmesi test edilecek. bir bilgisayar appi açtıktan sonra, diğer bir bilgisayar curl ile kendi senaryosunu upload edebiliyor mu?
- [ ]   senaryo yazımının basitleştirilmesi gerek

örneğin:
1) plotclockları kalibre et
2) plotclock working spacelerini çizdir
3) search algrotihm wombo jombo... -> sonuçta (x1,y1) (x2,y2) vs koordinatlar çıktığını düşünelim
4) plotclock no: 1 attack modda çalışsın gelen topu x1,y1 e atsın
5) plotclock no: 0 defence modda takılsın
6) top x2,y2 den geçerse plotclock no:1 topu x3,y3 e atsın, no:0 da x4,y4e atsın
gibi, gibi...

şu anda bizim kodumuzda plotclockların attack defence modları ayrı senaryolar halinde, her kullanıcıdan bunları baştan tanımlamasını bekleyemeyiz. 
hazır komutlar halinde sunmalıyız bazı şeyleri. onların kodunun çoğu search ve if else şeklinde olmalı
      
