# Projeto1-RoboticaComputacional
Entrega para o Projeto 1 de Robótica Computacional. <br />
Código prinpcial com as funções e comandos para o robô: main.py <br />
[Vídeo do funcionamento do robô](https://youtu.be/hs9sVoCzdCU) <br />
Alunos: André Weber e Lucas Muchaluat <br />


* O objeto a ser evitado pode ser escolhido via input limitado a uma lista pela rede neural utilizada (MobileNet Detection)
  * Contém um sistema de tracking para identificar o objeto escolhido somente após sua identificação em 5 frames seguidos.

* A cor a ser seguida pelo robô é azul, identificada pelo OpenCV.
  * Contém um controle proporcional para caso esteja se aproximando da cor. Assim, a medida que vai chegando mais perto da cor detectada pela câmera, vai diminuindo sua velocidade.

* O robô tem características de sobrevivência.
  * Um escaneamento infravermelho para evitar colisões com obstáculos próximos ao seu redor.
  * Bumpers para, caso haja colisão, se movimentar para desviar do obstáculo.
  
  


