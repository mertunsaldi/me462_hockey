Amaç: Modüler bir duvar sisteminin sensör ve actuatörleri için uygun bağlantı yöntemimi bulmak.

Gereklilikler:
- Karışıklılığı ve gelecekteki kullanımının kolaylaştırılması için kablo sayısının azaltılması.

Çözüm Önerileri:
1) Her sensör ve actuatöre micrprocessörden bir kablo çekmek (Kötü çözüm. her duvar arasında 15-20 pinlik bağlantı gerekir ve hangi sensörün hangi kabloya takıldığını kontrol etmek zor.)
2) Shift register kullanarak az bir bağlantı sayısıyla işi hallettmek.


2) Shift Register kullanımı
  -2 Çeşit shift resgister var piso (Parallel input serial output) ve sipo (Serial input parallel output)
  - Piso type birden fazla sensörü tek bir gpio pin ile okumaya yarıyor. Her bir clock sinyalinde paralel olarak yüklenmiş sinyal sırasıyla serial outputa iletiliyor.
      -Her bir duvarın içine bir adet piso type shift register konularak tüm sensörleri 3 kabloyla okuyabiliriz. (data, clock, load). 
  - Sipo type ile de birden fazla actuatör kontrol edilebilir. Maalesef pwm sinyali kullanılmak isteendiğinde manuel olarak oluşturulmak gerekecek.
      - Pwm kullanılmak istenirse PCA9685 16 Kanal i2c PWM/Servo Sürücü gibi bir sürücü kullanılabilir ama her gadget için bir tane kullanmak pahalı olabilir.
