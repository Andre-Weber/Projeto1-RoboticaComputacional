# Projeto1-RoboticaComputacional
Entrega para o Projeto 1 de Robótica Computacional. <br />
Código prinpcial com as funções e comandos para o robô: main.py <br />
[Vídeo do funcionamento do robô](https://youtu.be/hs9sVoCzdCU) <br />
Alunos: André Weber e Lucas Muchaluat <br />


* O objeto a ser evitado pode ser escolhido via input limitado a uma lista pela rede neural utilizada (MobileNet)
  * Contém um sistema de tracking para apenas identificar o objeto escolhido após certa quantidade de frames identificando.

* A cor a ser seguida pelo robô é azul, identificada pelo OpenCV.
  * Contém um controle proporcional para caso esteja perto diminua sua velocidade e caso esteja longe fique mais rápido.

* O robô tem características de sobrevivência.
  * Um escaneamento infravermelho para evitar colisões com obstáculos próximos.
  * Bumpers para caso haja batida se movimente a direção oposta.
  
  


