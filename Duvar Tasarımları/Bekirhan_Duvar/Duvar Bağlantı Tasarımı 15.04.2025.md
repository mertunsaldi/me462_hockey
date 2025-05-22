# Genel Güç Hattı
- 5V ve GND kabloları tüm duvarların içinden geçerek birbirleriyle bağlanacak.

# Actuatör Bağlantı Sistemi Tasarımı

## 1. Her actuatörün kendi mikroişlemcisi olacak.
- Mikroişlemci yalnızca ana bilgisayara bağlanacak.
- Tercihen sadece iki ekstra kablo ile bağlantı yapılabilecek (I2C ya da UART gibi; 5V ve GND zaten mevcut) ya da WiFi ile kablosuz bağlantı sağlanabilir (ancak hız yeterli olur mu???).

## 2. Her actuatör arenanın ana işlemcisi ile hareket ettirilecek.
- Kablolama daha karmaşık olacaktır, ancak SIPO tipi bir shift register kullanılabilir. Böylece tüm actuatörler minimum kablo sayısı ile kontrol edilebilir.
- SIPO ile PWM kontrolü verimli olmayabilir. Sinyalin manuel olarak PWM şeklinde oluşturulması gerekebilir.

## Sonuç
- Her actuatörün bir mikroişlemcisi olmasına karar verdik. Bunun sebebi, projenin bir eğitim ortamı oluşturmayı amaçlamasıdır. Böylece yeni bir gadget tasarımı, arena tasarımından mümkün olduğunca bağımsız şekilde yapılabilir.

## Mikroişlemciler Arasında Kablolu mu, Kablosuz Bir İletişim mi Kullanmalıyız?
- Her ne kadar hızlı bir top hareketi hedeflenmese de, topa vurma aksiyonunun gecikmeli olmasını istemiyoruz. Geçmiş deneyimlerden (örneğin ME461 projesi) WiFi’ın yeterli hıza ulaşamadığı anlaşılmış ve bu nedenle kablolu tasarım tercih edilmiştir.
- Kablolu haberleşme, WiFi seçeneğini tamamen dışlamaz. İstenirse her iki yöntem de uygulanabilir.

## Mikroişlemciler Arasında Hangi İletişim Protokolü Kullanılmalı?
- İlk başta I2C düşünülmüş, fakat yapılan araştırmalar sonucunda I2C’nin uzun kablolarda sorun çıkardığı öğrenilmiştir.
- UART da bir seçenek olarak değerlendirilebilir. Ayrıca yapay zekâ (AI) tarafından RS485 modülü kullanılması önerilmiştir.
- Nihayetinde, sadece 2 kablo ile tüm mikroişlemcileri birbirine bağlamak mümkün görünmektedir.

# Çarpma Sensörü Bağlantı Tasarımı

## Microswitch ya da 1/0 sinyali üreten başka bir birim kullanılacak.
- PISO tipi shift register kullanımı, 10.04.2025 tarihli toplantıda kabul edilmiştir.
- Load, Clock ve Data olmak üzere 3 kablo hattına ihtiyaç vardır.
- Bu hatlar aracılığıyla 1 ve 0 sinyali üreten diğer sensörler de okunabilir. Örneğin,daha önce konuşıulan içinden geçen topu temassız algılayan kızılötesi sensör de bu hatla okunabilir.

# Sonuç

- 5V, GND, Load, Clock, Data, ActuatorCom1 ve ActuatorCom2 olmak üzere minimum 7 adet kabloya ihtiyaç duyulmaktadır.

## T Şeklindeki Duvar Bağlantısı Sorunu

- Daisy-chain bağlantı yapısı tercih edildiğinden, bu yapı üçlü bağlantıyı (özellikle sensör veri kablosu için) desteklememektedir. Bu nedenle aşağıdaki alternatif çözüm yöntemleri değerlendirilmiştir:

### 1. İç Duvarlara Sensör Veri Kablosu Koymamak
- Adından da anlaşılacağı üzere, "sorunları sorun etmezsen sorun kalmaz" yaklaşımıdır.
- Ancak bu yaklaşım, iç duvarlardaki sensör verilerini alamamayı göze alır.

### 2. Sırt Sırta 2 Duvar Kullanarak İç Kısımlara Girmek
- Basit ve efektif bir yöntemdir.
- Ancak bu durumda ilgili bölgedeki duvar kalınlığı artacak ve bu durum simetrik bir duvar tasarımını desteklemeyebilir.

### 3. Sensör Veri Kablosu Sayısını Bir Adet Arttırıp Giriş-Çıkış Yapmak
- İç kısma giden duvarda iki veri kablosu kullanılıp, en uç noktada bu iki kablo birleştirilerek “U” yapısı oluşturulur.
- Bu şekilde toplamda 8 kabloya sahip olunur (bu sayı, konektör standardizasyonu açısından da daha uygun olabilir).
- Sıklıkla kullanılmayan bu ekstra kablo, gelecekte farklı amaçlarla da kullanılabilir. Örneğin:
  - İki duvarlı bir gadget tasarımında, bir mikroişlemci ile diğer duvardaki bir servo motor kontrol edilebilir.

##  Ada Şeklindeki Duvar Bağlantı Sorunu
- Daisy chain yapısı nedeniyle, arenanın dış duvarlarıyla fiziksel bağlantısı olmayan “ada” tipi duvarlar sıkıntı yaratmakta.
### 1. Ada duvarlarında gadget veya sensör olmasın
  - Düşünülebilir ama meh gibi de

### 2. Köprü denen bağlantı elemanı
  - Yalnızca kablo bağlantısı için duvarları birbirine bağlayan altından topun geçmesine müsade eden bir eleman
    
# Connector Seçimi
- Her duvarı birleştiren konnectör ve duvarın üstündeki gadget ve sensör bağlantı connectörü aynı cins olabilir böylece karşıklığın önüne geçmiş oluruz.
- Duvarla T çiziyorsak eğer her duvarı bir öncekine bağlamamız yeterli eğer ekstra data kablosu kullanılacaksa y yapılan yerdeki konnectörün fakrlı olması gerekecek.
- Her duvara çarpma sensörü eklenebielcekmiş gibi tasarım yaptığımızdan dolayı eğer duvarda sensör takılı değilse duvarın sağı ve soluna bağlanan data kablosu shortlanmalı. Eğer duvarda gadget ve sensör yoksa basit bir U şekilnde pin takılı kalabilir.
- Gadgetlar yalnızca 4 (eğer ekstra kablo da dahil edilecekse 5) pine ihtiyaç duyuyor. Duvarın üstündeki 8 pine geçebilen 4 pinli bir yapı düşünülebilir (OverEnginnering olabilir)
  
