import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns
import nltk
import string
import re
from sklearn.feature_extraction.text import CountVectorizer

df_articles = pd.read_csv("C:/Users/alexa/OneDrive/Área de Trabalho/CeDRI/PhD/State of Art/asreview2.csv")
'''
######################################### Publications by year ######################################################

ax = df_articles['Year'].value_counts().sort_index(ascending=True).plot(kind='bar', color='steelblue')

# Adicionar os valores acima das barras
for i, v in enumerate(df_articles['Year'].value_counts().sort_index(ascending=True)):
    ax.text(i, v, str(v), ha='center', va='bottom')

plt.xlabel('Year', fontsize='10')
plt.ylabel('Number of Publications', fontsize='10')

# Alterar a orientação do texto da legenda do eixo x para horizontal
ax.tick_params(axis='x', rotation=0)

# Mudar a cor da última barra para laranja
ax.patches[-1].set_facecolor('orange')

# Adicionar legenda para o número de artigos publicados até 17/05
legend_patch = mpatches.Patch(color='orange', label='Publications until \nMay 1st, 2023')
plt.legend(handles=[legend_patch], loc='upper left')

plt.savefig("Publications by year.png", dpi=1000)

#plt.show()

######################################### Publications by type ######################################################

ax = df_articles.groupby(['Year', 'Document Type'])['Year'].count().sort_index(ascending=True).unstack('Document Type').fillna(0).plot(kind='bar', stacked=True)
plt.xticks(fontsize='10', rotation=0)
ax.tick_params(axis='x', rotation=0)
plt.xlabel('Year', fontsize='10')
plt.ylabel('Number of Publications', fontsize='10')
for spine in ax.spines:
    ax.spines[spine].set_visible(True)

# Adicionando os valores no topo das barras
for container in ax.containers:
    for rect in container:
        height = rect.get_height()
        width = rect.get_width()
        x = rect.get_x()
        y = rect.get_y()
        ax.annotate(f'{int(height)}', (x + width/2, y + height), ha='center', va='bottom', fontsize='8')

# Ajustar a legenda
legend = ax.legend(loc='upper left', bbox_transform=plt.gcf().transFigure)
legend.set_title('Document Type')  # Título da legenda

plt.savefig('Publications by type and year.png', dpi=1000, bbox_inches='tight')
plt.show()'''

###################################### Publications by country ###############################################
'''
for index, row in df_articles.iterrows():
    author_affiliation = str(row['Affiliations']).split(';')
    print(author_affiliation)
    print('{} authors'.format(len(author_affiliation)))
    print('\n')
'''
'''
countries = pd.read_csv('C:/Users/alexa/OneDrive/Área de Trabalho/CeDRI/PhD/State of Art/countries.csv')

c = []
w = []
s = []
aux = ''

for index, row in df_articles.iterrows():
    peso = 100.0
    i = 0;
    sum = 0
    authors_affiliation = str(row['Affiliations']).split(';')
    for author in authors_affiliation:
      for country in countries['name']:
        if country in author:
          if author == aux:
            peso = peso + 10
          aux=author
          c.append({'Country': country})
          w.append({'Weight':peso})
          sum = sum + peso
          peso = peso - 10
          i = i + 1
    if sum != 0:
      for j in range(i):
        s.append({'Sum':sum})

df_countries = pd.concat([pd.DataFrame(c),pd.DataFrame(w),pd.DataFrame(s)], axis = 1)
df_countries['Contribution'] = (df_countries['Weight']/df_countries['Sum'])*100
df = (df_countries.groupby(['Country']).sum()/df_countries.groupby(['Country']).sum().sum())*100
df = df.sort_values(by=['Contribution'], ascending=False)
df = df.drop(['Weight', 'Sum'], axis=1)

plt.rcParams["figure.figsize"] = (8,6)
ax = df.head(22).sort_values(by='Contribution',ascending=True).plot.barh()
ax.legend().set_visible(False)
plt.xticks(fontsize=8, rotation=0)
plt.yticks(fontsize=8, rotation=0)
plt.xlabel('Contribution (%)', color='black',fontsize='8')
plt.ylabel('Country', color='black',fontsize='8')
for spine in ax.spines:
    ax.spines[spine].set_visible(True)
plt.savefig('Countries.png', bbox_inches='tight', dpi=600, format='png')
plt.show()
'''
'''
nltk.download('omw-1.4')
nltk.download('stopwords')
nltk.download('wordnet')

def clean_text(text):

    #stop word = useless data (e.g., the, a, an, in...)
    stopword = nltk.corpus.stopwords.words('english')

    #Grouping together the different inflected form of a word
    wn = nltk.WordNetLemmatizer()

    #Transform into lowercase and punctuation removal
    text = "".join([word.lower() for word in text if word not in string.punctuation])
    #print(text)
    
    #Tokenization
    tokens = re.split('\W+', text)
    #print(tokens)
  
    #Removel of stopwrads and lemmatizing
    text = " ".join([wn.lemmatize(word) for word in tokens if word not in stopword])
    #print(text)

    return text

def ngrams(text,n):
    
    ngram_vect = CountVectorizer(ngram_range=(n,n))
    
    X_counts = ngram_vect.fit_transform(text)
    
    X_counts_df = pd.DataFrame(X_counts.toarray())
    
    X_counts_df.columns = ngram_vect.get_feature_names_out()

    New_df= pd.DataFrame(columns=ngram_vect.get_feature_names_out())

    New_df = X_counts_df.sum(axis=0)

    return New_df

if df_articles.shape[0] > 1000:
  n = int(df_articles.shape[0]/1000)
else:
  n = 1
  
df_div = pd.DataFrame()
div = np.array_split(df_articles, n)

for i in range(n):
  div[i]['Title'] = div[i]['Title'].apply(lambda x: clean_text(str(x)))
  div[i]['Abstract'] = div[i]['Abstract'].apply(lambda x: clean_text(str(x)))
  div[i]['Author Keywords'] = div[i]['Author Keywords'].apply(lambda x: clean_text(str(x)))
  df_div = pd.concat([df_div, pd.DataFrame(div[i])], axis=0)

op = 4

if op == 1:
  ngram = 'unigrams'
elif op == 2:
  ngram = 'bigrams'
elif op == 3:
  ngram = 'trigrams'
else:
  ngram = 'ngrams'

if df_articles.shape[0] > 1000:
  n = int(df_articles.shape[0]/1000)
else:
  n = 1
  
div_2 = np.array_split(df_div, n)
ngrams_abstract = pd.Series(dtype='int64')
ngrams_title = pd.Series(dtype='int64')
ngrams_keywords = pd.Series(dtype='int64')

if op == 4:
  it = op-1
else:
  it = 1

for j in range(it):
  if op == 4:
    op2 = j + 1
  else:
    op2 = op
  for i in range(n):
    ngrams_abstract = pd.concat([ngrams_abstract, ngrams(div_2[i]['Abstract'],op2)], axis=0)
    ngrams_title = pd.concat([ngrams_title, ngrams(div_2[i]['Title'],op2)], axis=0)
    ngrams_keywords = pd.concat([ngrams_keywords, ngrams(div_2[i]['Author Keywords'],op2)], axis=0)

df_ngrams_abstract = pd.DataFrame({ngram:ngrams_abstract.index,'n':ngrams_abstract.values})
df_ngrams_title = pd.DataFrame({ngram:ngrams_title.index,'n':ngrams_title.values})
df_ngrams_keywords = pd.DataFrame({ngram:ngrams_keywords.index,'n':ngrams_keywords.values})
df_ngrams = pd.concat([df_ngrams_abstract,df_ngrams_title,df_ngrams_keywords])

df_ngrams_final = df_ngrams.groupby([ngram]).sum()
filter_ngram = df_ngrams_final[(df_ngrams_final['n']>2)]

filter_ngram.sort_values(by='n',ascending=False).to_excel('Dataset2_{}_{}_{}.xlsx'.format(ngram,np.sort(df_articles['Year'].unique())[0],np.sort(df_articles['Year'].unique())[-1]))

items = ['digital twin', 'simulation', 'data', 'machine learning', 'manufacturing', 'framework', 'realtime', 'network', 'reinforcement learning', 'optimization', 'simulation model', 'datadriven', 'artificial inteligence', 'diagnosis', 'scenario', 'whatif', 'industry 40']

df_ngrams_final = df_ngrams_final[df_ngrams_final.index.isin(items)]

df_ngrams_final

df_ngrams_final = df_ngrams_final.rename(index={'datadriven':'data-driven', 'whatif':'what-if', 'industry 40':'industry 4.0', 'realtime':'real-time'} )

df_ngrams_final = df_ngrams_final.groupby(df_ngrams_final.index).sum()

df_ngrams_final

plt.rcParams["figure.figsize"] = (8,6)

ax = df_ngrams_final.sort_values(by='n',ascending=True).plot.barh(width=0.5)
ax.legend().set_visible(False)
plt.xticks(fontsize=11, rotation=0)
plt.yticks(fontsize=11, rotation=0)
plt.xlabel('Frequency', color='black',fontsize='11')
plt.ylabel('', color='black',fontsize='11')
for spine in ax.spines:
    ax.spines[spine].set_visible(True)
plt.savefig('ngrams_{}.png'.format(ngram), bbox_inches='tight', dpi=600, format='png')
plt.show()'''

import matplotlib.pyplot as plt

# Dados de ocorrência
dados = {'Agricultural': 1, 'Formula 1': 1, 'Maintenance': 2, 'Manufacturing': 14, 'Microscopic Traffic': 1, 'Reinforcement learning': 1, 'Robotics': 4}

# Ordenar os dados por maior ocorrência
dados_ordenados = sorted(dados.items(), key=lambda x: x[1], reverse=True)

# Extrair os itens e as ocorrências ordenados
itens_ordenados = [item[0] for item in dados_ordenados]
ocorrencias_ordenadas = [item[1] for item in dados_ordenados]

# Calcular as porcentagens de ocorrência
total_ocorrencias = sum(ocorrencias_ordenadas)
porcentagens = [(ocorrencia / total_ocorrencias) * 100 for ocorrencia in ocorrencias_ordenadas]

# Plotar o gráfico de barras
plt.barh(range(len(itens_ordenados)), porcentagens, align='center')
plt.yticks(range(len(itens_ordenados)), itens_ordenados)
plt.xlabel('Occurencies (%)')
plt.ylabel('Fields')
#plt.title('Gráfico de Barras Horizontal - Ocorrências por Item')

plt.savefig('fields.png', bbox_inches='tight', dpi=1000)

plt.show()


