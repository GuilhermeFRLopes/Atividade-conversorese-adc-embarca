# Tarefa: Controle de LEDs RGB e Display SSD1306 com Joystick

## Descrição do Projeto
Este projeto implementa o controle de um LED RGB e a exibição de um quadrado móvel no display SSD1306 utilizando o Raspberry Pi Pico W e a placa BitDogLab. O sistema utiliza conversores analógico-digitais (ADC) para capturar os valores do joystick, ajustando dinamicamente o brilho dos LEDs e movimentando o quadrado no display.

Adicionalmente, o projeto emprega interrupções (IRQ) para capturar eventos dos botões, permitindo alternar o estado do LED verde e modificar a borda do display.



---

## Requisitos
1. **Controle dos LEDs RGB via Joystick:**  
   - LED Azul: Brilho proporcional ao eixo Y do joystick.
   - LED Vermelho: Brilho proporcional ao eixo X do joystick.
   - LEDs controlados via PWM para variação suave da intensidade luminosa
   
2. **Movimentação do Quadrado no Display SSD1306:**  
   - **O quadrado de 8x8 pixels deve se mover conforme os valores do joystick.**.  
   - **O display está conectado via I2C**.
     
3. **Funcionalidades dos Botões:**  
   - **Botão do Joystick (GPIO 22): Alterna o estado do LED Verde e modifica a borda do display**.
   - **Botão A (GPIO 5): Ativa ou desativa os LEDs PWM**.    
   - **Ambos os botões devem utilizar interrupções (IRQ) e possuir tratamento de debouncing**.   


---

## Configuração do Hardware
### Componentes Simulados no Wokwi

- **Raspberry Pi Pico W**.
- **Placa BitDogLab**.
- **LED RGB conectado às GPIOs 11, 12 e 13**.
- **Joystick conectado às GPIOs 26 (X) e 27 (Y)**.
- **Botão do Joystick na GPIO 22**.
- **Botão A na GPIO 5**.
- **Display SSD1306 conectado via I2C (GPIO 14 e 15)**. 

## Compilação e Execução

1. Clone o repositório do projeto.
2. Configure o ambiente de desenvolvimento do Raspberry Pi Pico.
3. Compile o código usando o CMake:
4. Carregue o binário no Raspberry Pi Pico.

###  Testando no Simulador
Se quiser testar no **Wokwi**, abra o arquivo `diagram.json` e inicie a simulação.

---

##  Link do video
 **Assista o video**:
https://youtu.be/l8wWqN2WEhM
 

---

##  Equipe de Desenvolvimento
| Nome | GitHub |
|------|--------|
| Guilherme Lopes | [@GuilhermeFRLopes](https://github.com/GuilhermeFRLopes) |


