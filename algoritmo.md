


			Toda vez que a diferença de posição atual e alvo de x ou y for diferente de 0 E o robô não estiver voando:
				- voar (aplicar z mínimo e mudar o status de voando pra True);
				Depois, vai realizar a movimentação:
					- REGRA/Plano de ordem de movimentação, pois z tem um limite espacial, precisa ter um plano de movimentação.
					
			Se z igual a 0 mudar o status de voando para False. 


​			
			REGRA: 
			- não modificar os valores angulares para grarantir uma mesma orientação entre movimentos/simulação. 
			- deixar a movimentação de altitude sempre em segundo plano: de modo que, sempre vai ser aplicado x e y, sem modificar z e depois z, sem modificar x e y.


​			
​			
​			
