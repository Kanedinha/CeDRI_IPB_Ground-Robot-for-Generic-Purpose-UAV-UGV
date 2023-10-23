# -*- coding: utf-8 -*-
import pandas as pd
import numpy as np
#import nltk
#nltk.download('stopwords')
#nltk.download('wordnet')
import re
import string
from sklearn.feature_extraction.text import CountVectorizer
from sklearn.feature_extraction.text import TfidfVectorizer
import Levenshtein

class data_clean():
    def __init__(self):
        ############################ Initialize Databases ##################################
        df_scopus = pd.read_csv("C:\Users\carlo\OneDrive\Área de Trabalho\Artigos State of Art/scopus.csv")
        df_wos = pd.read_csv("C:/Users/alexa/OneDrive/Área de Trabalho/CeDRI/PhD/State of Art/wos.txt", delimiter="\t")
        df_ieee = pd.read_csv("C:/Users/alexa/OneDrive/Área de Trabalho/CeDRI/PhD/State of Art/ieee.csv")

        ############################ Removendo colunas sem dados ##################################
        df_scopus = self.empty_columns(df_scopus)
        df_wos = self.empty_columns(df_wos)
        df_ieee = self.empty_columns(df_ieee)

        ############################ Renomeando colunas e removendo colunas sem interesse ##################################
        df_scopus = self.rename_scopus(df_scopus)
        df_wos = self.rename_wos(df_wos)
        df_ieee = self.rename_ieee(df_ieee)

        ############################ Removendo duplicate DOI papers ##################################
        df_wos = self.doi_duplicate(df_scopus, df_wos)      # Removendo duplicatas Scopus -> wos
        df_ieee = self.doi_duplicate(df_scopus, df_ieee)    # Removendo duplicatas Scopus -> IEEE
        df_ieee = self.doi_duplicate(df_wos, df_ieee)       # Removendo duplicates wos -> IEEE

        ############################ Removendo duplicate named papers ##################################
        df_scopus = self.clean_title(df_scopus)
        df_wos = self.clean_title(df_wos)
        df_ieee = self.clean_title(df_ieee)
        df_wos = self.title_duplicate(df_scopus, df_wos)      # Removendo duplicatas Scopus -> wos
        df_ieee = self.title_duplicate(df_scopus, df_ieee)    # Removendo duplicatas Scopus -> IEEE
        df_ieee = self.title_duplicate(df_wos, df_ieee)       # Removendo duplicates wos -> IEEE

        ############################ Removendo duplicate abstract papers ##################################
        df_scopus = self.clean_abstract(df_scopus)
        df_wos = self.clean_abstract(df_wos)
        df_ieee = self.clean_abstract(df_ieee)
        self.similar_abstracts(df_scopus, df_wos)
        self.similar_abstracts(df_scopus, df_ieee)
        self.similar_abstracts(df_wos, df_ieee)
        ########################## Encontrando titulos similares ######################################
        self.similar_titles(df_scopus, df_wos)
        self.similar_titles(df_scopus, df_ieee)
        self.similar_titles(df_wos, df_ieee)

        ########################## Removendo Note, Editorial, Retracted, Article in Press ######################################
        df_scopus = self.remove_types(df_scopus)
        df_wos = self.remove_types(df_wos)
        df_ieee = self.remove_types(df_ieee)

        ########################## Resetando index ######################################
        df_scopus = df_scopus.reset_index()
        df_wos = df_wos.reset_index()
        df_ieee = df_ieee.reset_index()

        df_final = pd.concat([df_scopus, df_wos, df_ieee])
        df_final = df_final.reset_index()
        df_final = df_final.drop(['index'], axis=1)
        df_final = df_final.drop(['level_0'], axis=1)
        df_final = df_final.rename(columns={'Column1': 'Index'})

        df_final['Year'] = df_final['Year'].fillna(0).astype(int)
        df_final['Cited by'] = df_final['Cited by'].fillna(0).astype(int)

        df_final.loc[655, 'Year'] = 2023
        df_final.loc[673, 'Year'] = 2022

        df_final.to_csv('Articles_database.csv')

        #print(df_scopus)
        #print(df_wos)
        #print(df_ieee)

    def remove_types(self, df):
        df = df.drop(df[df['Document Type'] == 'Retracted'].index)
        df = df.drop(df[df['Document Type'] == 'Editorial'].index)
        df = df.drop(df[df['Document Type'] == 'Note'].index)
        df = df.drop(df[df['Document Type'] == 'Wiley-IEEE Press eBook Chapters'].index)
        print(df['Document Type'].value_counts())

        return df

    def empty_columns(self, df):
        return df.dropna(axis='columns', how='all')

    def rename_scopus(self, df):
        return df[['Title', 'Authors', 'Abstract', 'Author Keywords', 'Year', 'Affiliations', 'Authors with affiliations',
                             'Source title', 'Document Type', 'Cited by', 'DOI']].copy()
    
    def rename_wos(self, df):
        df = df.rename(columns={'DI': 'DOI', 'TI': 'Title', 'AU': 'Authors', 'AB': 'Abstract', 'DE': 'Author Keywords',
                                        'PY': 'Year', 'SO': 'Source title', 'LA': 'Language', 'DT': 'Document Type',
                                        'CT': 'Conference_Title', 'TC': 'Cited_References', 'Z9': 'Cited_Reference_Count',
                                        'RP': 'Affiliations', 'C1': 'Authors with affiliations', 'U1': 'Cited by'})
        
        return df[['Title', 'Authors', 'Abstract', 'Author Keywords', 'Year', 'Affiliations', 'Authors with affiliations',
                             'Source title', 'Document Type', 'Cited by', 'DOI']].copy()

    def rename_ieee(self, df):
        df = df.rename(columns={'Document Title': 'Title', 'Author Keywords': 'Author Keywords', 
                                              'Publication Year': 'Year', 'Author Affiliations': 'Affiliations',
                                              'Publication Title': 'Source title', 'Document Identifier': 'Document Type',
                                              'Article Citation Count': 'Cited by'})
        
        df['Authors with affiliations'] = ''

        return df[['Title', 'Authors', 'Abstract', 'Author Keywords', 'Year', 'Affiliations', 'Authors with affiliations',
                             'Source title', 'Document Type', 'Cited by', 'DOI']].copy()

    def doi_duplicate(self, df1, df2):
        # Identify the Duplicates
        DOI_Paper_df1 = []
        DOI_Paper_df2 = []
        Index_paper_df1 = []
        Index_paper_df2 = []

        for index, row in df1.iterrows():
            for index_1, row_1 in df2.iterrows():
                if row_1["DOI"] == row["DOI"]:
                    DOI_Paper_df1.append(row["DOI"])
                    DOI_Paper_df2.append(row_1["DOI"])
                    Index_paper_df1.append(index)
                    Index_paper_df2.append(index_1)

        # Remove the duplicates
        return df2.drop(Index_paper_df2, axis=0)

    def clean_title(self, df):
        df['Title clean'] = df['Title'].str.lower()
        df['Title clean'] = df['Title clean'].apply(lambda x: re.sub(r'[^\w\s]', '', x).strip())
        return df
    
    def clean_abstract(self, df):
        df['Abstract clean'] = df['Abstract'].str.lower()
        df['abstract clean'] = df['Abstract clean'].apply(lambda x: re.sub(r'[^\w\s]', '', x).strip())
        return df

    def title_duplicate(self, df1, df2):
        # Identify the Duplicates
        DOI_Paper_df1 = []
        DOI_Paper_df2 = []
        Index_paper_df1 = []
        Index_paper_df2 = []

        for index, row in df1.iterrows():
            for index_1, row_1 in df2.iterrows():
                if row_1["Title clean"] == row["Title clean"]:
                    DOI_Paper_df1.append(row["Title clean"])
                    DOI_Paper_df2.append(row_1["Title clean"])
                    Index_paper_df1.append(index)
                    Index_paper_df2.append(index_1)

        # Remove the duplicates
        return df2.drop(Index_paper_df2, axis=0)

    def similar_titles(self, df1, df2):
        titles_df1 = df1['Title clean']
        titles_df2 = df2['Title clean']

        for index, title in titles_df1.iteritems():
            for index_1, title_1 in titles_df2.iteritems():
                similariedade = Levenshtein.ratio(title, title_1)
                if similariedade >= 0.8:
                    print(f"df1[{index}]: {title} - df2[{index_1}]: {title_1} (Similaridade: {similariedade})")

    def similar_abstracts(self, df1, df2):
        titles_df1 = df1['Abstract clean']
        titles_df2 = df2['Abstract clean']

        for index, title in titles_df1.iteritems():
            for index_1, title_1 in titles_df2.iteritems():
                similariedade = Levenshtein.ratio(title, title_1)
                if similariedade >= 0.5:
                    print(f"df1[{index}]: {title} - df2[{index_1}]: {title_1} (Similaridade: {similariedade})")

if __name__ == '__main__':
    data_clean()
