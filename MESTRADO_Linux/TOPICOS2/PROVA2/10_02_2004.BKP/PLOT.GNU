# Script para o GNU Plot
# Suess, Claro e Kühne
# Tópicos II
# Janeiro 2004
#
# usado em nodes.cpp
#
# coluna 1: x real
# coluna 2: y real
# coluna 3: theta real
# coluna 4: x estimado
# coluna 5: 7 estimado
# coluna 6: theta estimado
# coluna 7: ruído
# colocar coluna 8: tempo

reset

set title "Trajetória em 8"
set xlabel "eixo x"
set ylabel "eixo y"
set grid

# eixo x: coluna 1 (x); eixo y: coluna 2 (y)
plot 'saida.dat' u 1:2 title "trajetória real" w lines
#plot 'saida.dat' u 7:2 title "trajetória estimada" w lines

set output 'saida.eps'

# para manter a janela do gráfico aberta:
pause -1

# Fim



