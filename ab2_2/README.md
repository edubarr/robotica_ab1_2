# Robotic_AB2_Part2

## Questão 1:

Temos que a tabela DH para um manipulador planar RR é:

| j   | θⱼ  | dⱼ  | aⱼ  | ⍺ⱼ   |
| --- | --- | --- | --- | ---- |
| 1   | θ1  | 0   | L1  | 0.0° |
| 2   | θ2  | 0   | L2  | 0.0° |

Assim, podemos calcular analíticamente a Pose final pela transformação:

$`^jT_{j+1}=\begin{bmatrix}\cos \theta _j&-\sin \theta _j\cos \alpha _j&\sin \theta _j\sin \alpha _j&a_j\cos \theta _j\\
\sin \theta _j&\cos \theta _j\cos \alpha _j&-\cos \theta _j\sin \alpha _j&a_j\sin \theta _j\\
0&\sin \alpha _j&\cos \alpha _j&d_j\\
0&0&0&1\end{bmatrix}`$

Onde temos que a Pose final será dada por:

$`^0T_2=^0T_1\cdot^1T_2`$

$`^0T_2=\begin{bmatrix}Cos(\theta1)&-Sen(\theta1)&0&L_1Cos(\theta1)\\
Sen(\theta1)&Cos(\theta1)&0&L_1Sen(\theta1)\\
0&0&1&0\\
0&0&0&1\end{bmatrix}\begin{bmatrix}Cos(\theta2)&-Sen(\theta2)&0&L_2Cos(\theta2)\\
Sen(\theta2)&Cos(\theta2)&0&L_2Sen(\theta2)\\
0&0&1&0\\
0&0&0&1\end{bmatrix}`$

$`^0T_2=\begin{bmatrix}
cos(\theta1)*cos(\theta2) - sin(\theta1)*sin(\theta2) & sin(\theta1)*(-cos(\theta2)) - cos(\theta1)*sin(\theta2) & 0 & -L2 sin(\theta1) sin(\theta2) + L2 cos(\theta1) cos(\theta2) + L1 cos(\theta1)\\
sin(\theta1)*cos(\theta2) + cos(\theta1)*sin(\theta2) & cos(\theta1)*cos(\theta2) - sin(\theta1)*sin(\theta2) & 0 & L2*sin(\theta1)*cos(\theta2) + L2*cos(\theta1)*sin(\theta2) + L1*sin(\theta1)\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1\end{bmatrix}`$

Sendo assim, podemos concluir que x e y são:

$`x =L1*cos(\theta1) + L2*cos(\theta1 + \theta2)`$

$`y = L1*sin(\theta1) + L2*sin(\theta1 + \theta2)`$

Então, de forma analitica podemos calcular a Jacobiana para esse manipulador:

$`J(q) = \frac{\partial}{\partial(q)}K(q)=\begin{bmatrix}\frac{\partial x}{\partial(θ1)}&\frac{\partial x}{\partial(θ2)}\\\frac{\partial y}{\partial(θ1)}&\frac{\partial y}{\partial(θ2)}\end{bmatrix}`$

Derivando:

$`\frac{\partial x}{\partial(θ1)} =  -L1*sin(θ1) - L2*sin(θ2 + θ1)`$

$`\frac{\partial x}{\partial(θ2)} = -L2*sin(θ2 + θ1)`$

$`\frac{\partial y}{\partial(θ1)} = L1*cos(θ1) + L2*cos(θ2 + θ1)`$

$`\frac{\partial y}{\partial(θ2)} =  L2*cos(θ2 + θ1)`$

$`J(q) =\begin{bmatrix}-L1*sin(θ1) - L2*sin(θ2 + θ1)&-L2*sin(θ2 + θ1)\\ L1*cos(θ1) + L2*cos(θ2 + θ1)&  L2*cos(θ2 + θ1)\end{bmatrix}`$

### A)

Utilizando a robotics-toolbox de Peter Corke, foi possível modelar o braço planar RR utilizando a classe DHRobot utilizando a função rr_robot() (arquivo [P1.py](P1.py))

A trajetória entre TE1 e TE2 pode ser canculada utilizando a função ctraj. Essa trajetória é composta por um array contendo todas as poses em um determinado período de tempo entre TE1 e TE2.

Tendo o array de trajetória, podemos utilizar a matriz Jacobiana encontrada anteriormente para implementar o algoritmo **resolved rate control**, para, utilizando as trajetórias e velocidades das juntas, reduzir o erro e integrar os ângulos das juntas até a pose final desejada. Dessa forma, podemos encontrar os valores dos ângulos das juntas, sem ser necessário realizar o cálculo da cinemática inversa do manipulador. O algoritmo foi implementado na função resolved_rate_control_2r() (arquivo [P1.py](P1.py))

**Temos os ângulos finais das juntas para a Pose (setup.py):**

```
q = [-0.9182050704747622, 1.7654030627350557]
```

Podemos notar algumas diferencças de se utilizar este método em relação ao cálculo da cinemática inversa analiticamente. No caso do algoritmo implementado, temos apenas uma solução, pois o mesmo utiliza de um conceito de sistema de controle, onde ele busca minimizar o erro para encontrar os ângulos desejados, dessa forma encontrando apenas o conjunto de ângulos mais próximos da pose inicial. Outro ponto a ser considerado é a presença de "pontos de singularidades", nas quais são pontos que impede que o manipulador alcance a pose especificada, e que o algoritmo não considera isso, sendo necessário alguma implementação a mais para impedir tais casos.

Para melhor a aproximação podemos modificar o parâmetro Lambda ou aumentar o tempo para que o erro diminua.

### B)

A matriz Jacobiana é utilizada para descrever o impacto na saída de mudanças nas variáveis de um sistema, sendo utilizada para análise de sistemas dinâmicos. A sua inversa é muito utilizada para encontrar as mudanças nas variáveis que acarretam a uma mudança desejada na saída. Já a sua transposta apenas reorganiza as entradas da matriz, ou seja, não possui a mesma utilizada da inversa, onde nesse caso não iria encontrar os valores desejados dos ângulos para as juntas.

### C)

Podemos utilizar a função implementada plot_p1() (arquivo [P1.py](P1.py)), para plotar os gráficos de erro de posição e das juntas em relação ao tempo, obtendo os seguintes resultados:

**Saída:**

<div style="display: flex;">
  <a name="figura1"></a>
  <img src="imgs/P1_1.png" alt="q+" style="width: 47%;">
  <a name="figura2"></a>
  <img src="imgs/P1_2.png" alt="" style="width: 45%;">
</div>

Podemos verificar que, conforme o tempo passa e as juntas vão se posicionando na posição desejada, o erro vai diminuindo.

## Questão 2:
