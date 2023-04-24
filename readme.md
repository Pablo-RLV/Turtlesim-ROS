# Turtlesim: simulando um ambiente robótico integrado no ROS

## 1. Introdução

O seguinte projeto foi desenvolvido durante o terceiro semestre do curso Engenharia da Computação no Inteli - Instituto de Tecnologia e Liderança. Seu objetivo é trazer aprendizado sobre o ROS - Robot Operating System, um framework de código aberto para desenvolvimento de robôs. Para isso, foi utilizado o simulador Turtlesim, que simula um ambiente robótico com uma tartaruga que pode ser controlada através de um script em Python.

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
chmod +x src/tortuguita.py
```

2.2. Ainda na pasta do projeto, execute o seguinte comando:

```bash
./tortuguita.py
```

## 4. Resultados

O resultado final esperado é o seguinte:

<center>
    <img src="/imgs/exemplo.png" width="200px" height="200px">
</center>