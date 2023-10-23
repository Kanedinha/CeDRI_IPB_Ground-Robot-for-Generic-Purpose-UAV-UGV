# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
import glob

######################################## Scopus ########################################

diretorio = "D:/Kaneda/IPB/Tese/Ground Robot/state of art_Ground Robot/scopus"

# Lista todos os arquivos CSV no diretório
arquivos_csv = glob.glob(diretorio + "/*.csv")

# Inicializa um DataFrame vazio para armazenar os dados combinados
df_scopus = pd.DataFrame()
df_temp = pd.DataFrame()

# df_scopus = pd.read_csv(diretorio)
    
#     # Remove colunas com todos os valores NULL
# df_scopus = df_scopus.dropna(axis='columns', how='all')
    
#     # Remove linhas sem autores
# df_scopus = df_scopus.dropna(subset=['Authors'])


# Loop para ler e combinar os arquivos CSV
for arquivo in arquivos_csv:
    # Lê o arquivo CSV
    df_temp = pd.read_csv(arquivo)
    
    # Remove colunas com todos os valores NULL
    df_temp = df_temp.dropna(axis='columns', how='all')
    
    # Remove linhas sem autores
    df_temp = df_temp.dropna(subset=['Authors'])
    
    # Adiciona o DataFrame temporário ao DataFrame combinado
    df_scopus = pd.concat([df_scopus, df_temp], ignore_index=True)

# Reset o índice do DataFrame combinado
df_scopus = df_scopus.reset_index()

# Exibe o DataFrame combinado
print("scopus:")
print(df_scopus)

########################################## WoS #########################################

# df_wos = pd.read_csv("D:/Kaneda/IPB/Tese/Ground Robot/state of art_Ground Robot/wos/wos.txt", delimiter="\t")

# # Concatenating dataframes into a single dataframe
# # df_wos = pd.concat([df_wos_1, df_wos_2], axis=0)

# # Remove all colums with all values NULL
# df_wos = df_wos.dropna(axis='columns', how='all')

# # Change the header name
# df_wos = df_wos.rename(columns={'DI': 'DOI', 'TI': 'Title', 'AU': 'Authors', 'AB': 'Abstract', 'DE': 'Author_Keywords', 'PY': 'Year',
#                                 'SO': 'Source_Title', 'LA': 'Language', 'DT': 'Document_Type', 'CT': 'Conference_Title',
#                                 'TC': 'Cited_References', 'Z9': 'Cited_Reference_Count'})

# # Remove all rows without authors
# # df_wos = df_wos.dropna(subset=['Authors'])

# # Remove all papers without DOI
# # df_wos = df_wos.dropna(subset=['DOI'])

# # Reset the index of the dataframe
# df_wos = df_wos.reset_index()

# # Drop the old index column
# df_wos = df_wos.drop(['index'], axis=1)

######################################### IEEE #########################################

def processar_arquivo(nome_arquivo):
    # Lê o arquivo CSV
    df = pd.read_csv(nome_arquivo)
    
    # Remove colunas com todos os valores NULL
    df = df.dropna(axis='columns', how='all')
    
    # Remove linhas sem autores
    df = df.dropna(subset=['Authors'])
    
    # Remove papers sem DOI
    df = df.dropna(subset=['DOI'])
    
    # Reset o índice do DataFrame
    df = df.reset_index(drop=True)
    
    # Substitui espaços nos nomes das colunas por _
    df.columns = df.columns.str.replace(' ', '_')
    
    return df

# Diretório onde os arquivos CSV estão localizados
dir = "D:/Kaneda/IPB/Tese/Ground Robot/state of art_Ground Robot/ieee"

# Lista todos os arquivos CSV no diretório
arquivos_csv = glob.glob(dir + "/*.csv")

# Inicializa um DataFrame vazio para armazenar os dados combinados
df_ieee = pd.DataFrame()

# Loop para processar e combinar os arquivos CSV
for arquivo in arquivos_csv:
    df_temp = processar_arquivo(arquivo)
    df_ieee = pd.concat([df_ieee, df_temp], ignore_index=True)

# Exibe o DataFrame combinado
print(df_ieee)

#################################### Cleaning Dataset ####################################

# # Identify the Duplicates of SCOPUS -> WOS

# DOI_Paper_scopus = []
# DOI_Paper_wos = []
# Index_paper_scopus = []
# Index_paper_wos = []

# for index, row in df_scopus.iterrows():
#     for index_1, row_1 in df_wos.iterrows():
#         if row_1["DOI"] == row["DOI"]:
#             DOI_Paper_scopus.append(row["DOI"])
#             DOI_Paper_wos.append(row_1["DOI"])
#             Index_paper_scopus.append(index)
#             Index_paper_wos.append(index_1)

# # Remove the duplicates of SCOPUS -> WoS
# df_wos = df_wos.drop(Index_paper_wos, axis=0)
# print(df_wos)

# df_wos.to_csv('wos_new.csv')

# Identify the Duplicates of SCOPUS -> IEEE

DOI_Paper_scopus = []
DOI_Paper_IEEE = []
Index_paper_scopus = []
Index_paper_IEEE = []

for index, row in df_scopus.iterrows():
    for index_1, row_1 in df_ieee.iterrows():
        if row_1["DOI"] == row["DOI"]:
            DOI_Paper_scopus.append(row["DOI"])
            DOI_Paper_IEEE.append(row_1["DOI"])
            Index_paper_scopus.append(index)
            Index_paper_IEEE.append(index_1)

# # Remove the duplicates of SCOPUS -> IEEE
df_ieee = df_ieee.drop(Index_paper_IEEE, axis=0)
print(df_ieee)

df_ieee.to_csv('ieee_new.csv')

# # Identify the Duplicates of WoS-> IEEE

# DOI_Paper_wos = []
# DOI_Paper_IEEE = []
# Index_paper_wos = []
# Index_paper_IEEE = []

# for index, row in df_wos.iterrows():
#     for index_1, row_1 in df_ieee.iterrows():
#         if row_1["DOI"] == row["DOI"]:
#             DOI_Paper_wos.append(row["DOI"])
#             DOI_Paper_IEEE.append(row_1["DOI"])
#             Index_paper_wos.append(index)
#             Index_paper_IEEE.append(index_1)

# # Remove the duplicates of WoS -> IEEE
# df_ieee = df_ieee.drop(Index_paper_IEEE, axis=0)
# print(df_ieee)

# df_ieee.to_csv('ieee_new.csv')

#################################### Final Dataset ####################################
df_scopus_final = df_scopus[['Title', 'Authors', 'Year', 'Source title', 'Document Type', 'Cited by']].copy()

# df_wos_final = df_wos[['Title', 'Authors', 'Abstract', 'Author_Keywords', 'Year', 'RP', 'C1', 'Source_Title', 'Document_Type',
#                        'U1']].copy()

df_ieee['DT'] = df_ieee['Document_Identifier'] #.str.replace('IEEE', '')
df_ieee['Affiliations'] = ''


df_ieee_final = df_ieee[['Document_Title', 'Authors', 'Abstract', 'Author_Keywords', 'Publication_Year', 'Affiliations', 'Author_Affiliations', 'Publication_Title', 'DT', 'Article_Citation_Count']].copy()

# df_wos_final = df_wos_final.rename(columns={'Author_Keywords': 'Author Keywords', 'RP': 'Affiliations',
#                                             'C1': 'Authors with affiliations', 'Source_Title': 'Source title',
#                                             'Document_Type': 'Document Type', 'U1': 'Cited by'})

df_ieee_final = df_ieee_final.rename(columns={'Document_Title': 'Title', 'Author_Keywords': 'Author Keywords', 
                                              'Publication_Year': 'Year', 'Author_Affiliations': 'Authors with affiliations',
                                              'Publication_Title': 'Source title', 'DT': 'Document Type',
                                              'Article_Citation_Count': 'Cited by'})

# df_final = pd.concat([df_scopus_final, df_wos_final, df_ieee_final], axis=0)
df_final = pd.concat([df_scopus_final, df_ieee_final], axis=0)

df_final = df_final.reset_index()

df_final = df_final.drop(['index'], axis=1)

# Manually adding missing years
df_final.loc[671, 'Year'] = 2023
df_final.loc[692, 'Year'] = 2022

df_final['Year'] = df_final['Year'].fillna(0).astype(int)
df_final['Cited by'] = df_final['Cited by'].fillna(0).astype(int)

print(df_final)

df_final.to_csv('D:/Kaneda/IPB/Tese/Ground Robot/state of art_Ground Robot/Robozaum.csv')

count = df_final["Document Type"].value_counts()
print(count)
