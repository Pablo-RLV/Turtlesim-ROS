# Turtlesim: simulando um ambiente robótico integrado no ROS

## 1. Introdução

O seguinte projeto foi desenvolvido durante o terceiro semestre do curso Engenharia da Computação no Inteli - Instituto de Tecnologia e Liderança. Seu objetivo é trazer aprendizado sobre o ROS - Robot Operating System, um framework de código aberto para desenvolvimento de robôs. Para isso, foi utilizado o simulador Turtlesim, que simula um ambiente robótico com uma tartaruga que pode ser controlada através de um script em Python.

[![Turtlesim](https://youtube.com/shorts/icUqVEkYM6k?feature=share)](https://youtube.com/shorts/icUqVEkYM6k?feature=share)

Link para o vídeo do YouTube: <https://youtube.com/shorts/icUqVEkYM6k?feature=share>

## 2. Instalação

Para realizar a instalação do que será necessário para o prosseguimento desse projeto, siga os passos detalhados no readme desse projeto: <https://github.com/Murilo-ZC/Questoes-Trabalhos-Inteli-M6/tree/main/ponderada1>

## 3. Execução

Para executar o projeto, siga os seguintes passos:

1. Abra um terminal e execute o seguinte comando:

```bash
ros2 run turtlesim turtlesim_node
```

2.1. Em outra janela do terminal, entre na pasta do projeto e execute o seguinte comando:

```bash
chmod +x tortuguita.py
```

2.2. Ainda na pasta do projeto, execute o seguinte comando:

```bash
./tortuguita.py
```

## 4. Resultados

O resultado final esperado é o seguinte:

<center>
    <img src="/media/exemplo.png" width="200px" height="200px">
</center>

## 5. Features

- [x] Movimentação da tartaruga
- [x] Mudança de cor da linha desenhada pela tartaruga
- [x] Mudança de cor do plano de fundo
- [x] Formação de desenho personalizável

## 6. Extra

Como adição extra, foi inserido o script em python "espira.py", que permite a criação de uma espiral colorida com a tartaruga. O processo de execução é o mesmo do outro script apresentado anteriormente. O resultado final esperado é o seguinte:

<center>
    <img src="/media/espira.png" width="200px" height="200px">
</center>