# Fourier-and-Hartley-transforms-for-embedded-applications
Códigos utilizados para meu TCC, Engenharia mecatrônica, Unifran, 2016.
* Segue a descrição dos diretórios contidos neste repositório:
* [Dev_cpp](https://github.com/JunioCesarFerreira/Fourier-and-Hartley-transforms-for-embedded-applications/tree/master/Dev_cpp): 
Códigos para transformadas discretas e dos algoritmos Cooley-Tukey Radix-2 para as transformadas rápidas de Fourier e de Hartley, este é um arquivo interessante para quem está procurando compreender o funcionamento destes algoritmos. Desenvolvido em linguagem C no DEV-CPP.
* [ESP8266_Arduino](https://github.com/JunioCesarFerreira/Fourier-and-Hartley-transforms-for-embedded-applications/tree/master/ESP8266_Arduino): 
Essencialmente os mesmos códigos desenvolvidos em C do diretório descrito acima, porém, prontos para rodar no NodeMCU ESP8266 com comunicação USB. No diretória de 'Interfaces' encontra-se o programa de recepção dos dados gerados pelo ESP8266.
* [Firmware_Vibration_Analiser](https://github.com/JunioCesarFerreira/Fourier-and-Hartley-transforms-for-embedded-applications/tree/master/Firmware_Vibration_Analiser):
Neste encontram-se as duas versões do código do analisador de vibrações mecânicas, sendo que a única diferença entre as duas é que na segunda (Versão 3) foi habilitada uma saida para testes em sistemas discretos, tais como o exemplo apresentado no trabalho escrito como aplicação de um delta de Dirac no AutoFalante. Veja [Trabalho_Escrito](https://github.com/JunioCesarFerreira/Fourier-and-Hartley-transforms-for-embedded-applications/blob/Trabalho_Escrito/TCC_Junio.pdf).
* [Interfaces](https://github.com/JunioCesarFerreira/Fourier-and-Hartley-transforms-for-embedded-applications/tree/master/Interfaces):
Neste encontram-se duas interfaces desenvolvidas em linguagem C# no Microsoft Visual Studio, sendo:
- 'MyDataReceiver': Interface que recebe dados via USB, desenvolvida para receber os dados enviados pelo ESP [ESP8266_Arduino](https://github.com/JunioCesarFerreira/Fourier-and-Hartley-transforms-for-embedded-applications/tree/master/ESP8266_Arduino)
- 'AnalisadorTresEixos - V2': Interface desenvolvida para o analisador de vibrações mecânicas, recebe e plota os dados enivados pelo ESP com o programa [Firmware_Vibration_Analiser](https://github.com/JunioCesarFerreira/Fourier-and-Hartley-transforms-for-embedded-applications/tree/master/Firmware_Vibration_Analiser). Permitindo inclusive, salvar dados em planilhas para tratamento posterior.
* [Matlab](https://github.com/JunioCesarFerreira/Fourier-and-Hartley-transforms-for-embedded-applications/tree/master/Matlab):
Alguns dos códigos de testes em Matlab, uteis para plotar gráficos do trabalho e para verificar os resultados das transformadas. 
* [PIC18F](https://github.com/JunioCesarFerreira/Fourier-and-Hartley-transforms-for-embedded-applications/tree/master/PIC18F):
Versão desenvolvida no MikroC para PIC18F2550/4550. 
