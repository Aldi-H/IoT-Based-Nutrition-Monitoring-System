## Installation

- Install PlatformIO extension on VSCode

- Fork this repositories

- Clone ypur fork repositories

```bash
  git clone [YourGitHubForkRepositories]
  cd IoT-Based-Nutrition-Monitoring-System
```

- Wait for PlatformIO finished install depedencies

- Open .pio -> libdeps -> GravityTDS -> GravityTDS.cpp and edit .cpp file to

```c++
GravityTDS::GravityTDS()
{
   this->pin = 34;             // Based on your pin
   this->temperature = 25.0;
   this->aref = 3.0;           // Reference voltage on ADC
   this->adcRange = 4095.0;    // 1024 for 10bit ADC;4096 for 12bit ADC
   this->kValueAddress = 8;
   this->kValue = 1.0;
}
```

- ### Happy Coding
