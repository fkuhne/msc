%==============================================================================
% deletex - Documents for DELET/EE/UFRGS
% Copyright (C) 2004 Walter Fetter Lages <w.fetter@ieee.org>
%
% This class was adapted from:
% $Id: iiufrgs.cls,v 4.14 2003/11/21 22:36:13 avila Exp $
% Copyright (C) 2001-2003 UFRGS TeX Users Group
%
% This program is free software; you can redistribute it and/or modify
% it under the terms of the GNU General Public License as published by
% the Free Software Foundation; either version 2 of the License, or
% (at your option) any later version.
%
% This program is distributed in the hope that it will be useful,
% but WITHOUT ANY WARRANTY; without even the implied warranty of
% MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
% GNU General Public License for more details.
%
% You should have received a copy of the GNU General Public License
% along with this program; if not, write to the Free Software
% Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
%
%==============================================================================

%==============================================================================
% Identificação
%==============================================================================
\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{deletex}[2004/11/29 v0.4.0 DeleTeX class - Documents for DELET/EE/UFRGS]

%==============================================================================
% Declaração de opções
%==============================================================================
\newcommand{\deletexu@novodoc}[1]{\DeclareOption{#1}{\AtEndOfClass{\input{deletex#1.def}}}}
\deletexu@novodoc{rp}
\deletexu@novodoc{prop-tese}
\deletexu@novodoc{plano-doutorado}
\deletexu@novodoc{dipl-ecp}
\deletexu@novodoc{dipl-ele}
\deletexu@novodoc{ti}
\deletexu@novodoc{pep}
\deletexu@novodoc{diss}
\deletexu@novodoc{tese}
\deletexu@novodoc{tc}
\deletexu@novodoc{espec}
\DeclareOption{english}{\@englishtrue\OptionNotUsed}\newif\if@english
\DeclareOption{relnum}{\@relnumtrue\OptionNotUsed}\newif\if@relnum
\DeclareOption{oneside}{\AtEndOfClass{\@twosidefalse}\OptionNotUsed}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{report}}
\ProcessOptions\relax

%==============================================================================
% Herança da classe `report' e inclusão de pacotes e arquivos necessários
%==============================================================================
\LoadClass[12pt,a4paper,twoside]{report}
\RequirePackage[english,brazilian]{babel}
\input{deletex_pt_BR.ldf}
\input{deletex_en.ldf}
\if@english
        \selectlanguage{english}
        \AtBeginDocument{\selectlanguage{english}}
\fi
\RequirePackage{lastpage}
\RequirePackage{indentfirst}
\RequirePackage[T1]{fontenc}
\RequirePackage{times}
\RequirePackage{deletexdefs}
\RequirePackage{natbib}
%==============================================================================
% Margens do texto
%==============================================================================
\newlength{\deletex@topmargin}\setlength{\deletex@topmargin}{30mm}
\newlength{\deletex@pagenummargin}\setlength{\deletex@pagenummargin}{20mm}
\newlength{\deletex@bottommargin}\setlength{\deletex@bottommargin}{20mm}
\newlength{\deletex@sidemargin}\setlength{\deletex@sidemargin}{30mm}
\setlength{\marginparwidth}{25mm}

% ajuste das medidas verticais
\setlength{\topmargin}{\deletex@pagenummargin}
        \addtolength{\topmargin}{-1em} % isto é +- a altura do núm. da página
        \addtolength{\topmargin}{-1in}
\setlength{\headheight}{1em}
\setlength{\headsep}{\deletex@topmargin}
        \addtolength{\headsep}{-\deletex@pagenummargin}
\setlength{\textheight}{\paperheight}
        \addtolength{\textheight}{-\deletex@topmargin}
        \addtolength{\textheight}{-\deletex@bottommargin}
\setlength{\footskip}{\deletex@bottommargin}

% ajuste das medidas horizontais
\setlength{\oddsidemargin}{\deletex@sidemargin}
        \addtolength{\oddsidemargin}{-1in}
\setlength{\evensidemargin}{\oddsidemargin}
\setlength{\textwidth}{\paperwidth}
        \addtolength{\textwidth}{-2\deletex@sidemargin}
\setlength{\marginparsep}{\deletex@sidemargin}
        \addtolength{\marginparsep}{-\marginparwidth}
        \setlength{\marginparsep}{.5\marginparsep}

%==============================================================================
% Numeração das páginas
%==============================================================================
\def\ps@deletex{
        \let\@oddfoot\@empty
        \let\@evenfoot\@empty
        \def\@evenhead{{\small\thepage}}
        \def\@oddhead{\hspace*{\fill}{\small\thepage}}
}
\pagestyle{empty}
\let\ps@plain\ps@empty

%==============================================================================
% Definição de autor(es), orientador, co-orientador e banca
%==============================================================================
\newcounter{deletex@authorcount}
\renewcommand{\author}[2]{
        \addtocounter{deletex@authorcount}{1}
        \expandafter\gdef\csname @author\alph{deletex@authorcount}last\endcsname{#1}
        \expandafter\gdef\csname @author\alph{deletex@authorcount}first\endcsname{#2}
        \expandafter\gdef\csname @author\alph{deletex@authorcount}\endcsname{#2 #1}
}
\DeclareRobustCommand{\advisor}[3][]{
        \gdef\@advisorlast{#2}
        \gdef\@advisorfirst{#3}
        \gdef\@advisor{#1~#3 #2}
}
\newcommand{\advisorinfo}[2]{%
	\gdef\@advisorinstitution{#1}
	\gdef\@advisortitleinfo{#2}
}

\newcommand{\advisorwidth}[1]{\gdef\@advisorwidth{#1}}
\advisorwidth{0.55\textwidth}

\DeclareRobustCommand{\coadvisor}[3][]{
        \gdef\@coadvisorlast{#2}
        \gdef\@coadvisorfirst{#3}
        \gdef\@coadvisor{#1~#3 #2}
}
\newcommand{\coadvisorinfo}[2]{
	\gdef\@coadvisorinstitution{#1}
	\gdef\@coadvisortitleinfo{#2}
}

\newcounter{deletex@examinercount}
\DeclareRobustCommand{\examiner}[3][]{
        \addtocounter{deletex@examinercount}{1}
        \expandafter\gdef\csname @examiner\alph{deletex@examinercount}last\endcsname{#2}
        \expandafter\gdef\csname @examiner\alph{deletex@examinercount}first\endcsname{#3}
        \expandafter\gdef\csname @examiner\alph{deletex@examinercount}\endcsname{#1~#3 #2}
}
\newcounter{deletex@examinerinfocount}
\newcommand{\examinerinfo}[2]{
        \addtocounter{deletex@examinerinfocount}{1}
        \expandafter\gdef\csname @examiner\alph{deletex@examinerinfocount}institution\endcsname{#1}
        \expandafter\gdef\csname @examiner\alph{deletex@examinerinfocount}titleinfo\endcsname{#2}
}

%==============================================================================
% Definição do Departamento relacionado ao documento
% deve ser definido como `DELET'
%==============================================================================
\newcommand{\dept}[1]{\gdef\@dept{#1}}
\def\DELET{\delet}
\dept{DELET}

%==============================================================================
% Definição das palavras-chave (máx. 10)
%==============================================================================
\newcounter{kwcounter}
\newcommand{\keyword}[1]{
        \addtocounter{kwcounter}{1}
        \expandafter\gdef\csname keyw\alph{kwcounter}\endcsname{#1}
}
\newcommand{\@cipkw}{%
        \@ifundefined{keywa}{}{1.~\expandafter\MakeUppercase\keywa\@. }%
        \@ifundefined{keywb}{}{2.~\expandafter\MakeUppercase\keywb\@. }%
        \@ifundefined{keywc}{}{3.~\expandafter\MakeUppercase\keywc\@. }%
        \@ifundefined{keywd}{}{4.~\expandafter\MakeUppercase\keywd\@. }%
        \@ifundefined{keywe}{}{5.~\expandafter\MakeUppercase\keywe\@. }%
        \@ifundefined{keywf}{}{6.~\expandafter\MakeUppercase\keywf\@. }%
        \@ifundefined{keywg}{}{7.~\expandafter\MakeUppercase\keywg\@. }%
        \@ifundefined{keywh}{}{8.~\expandafter\MakeUppercase\keywh\@. }%
        \@ifundefined{keywi}{}{9.~\expandafter\MakeUppercase\keywi\@. }%
        \@ifundefined{keywj}{}{10.~\expandafter\MakeUppercase\keywj\@. }%
}
\newcommand{\@abstractkw}{%
        \@ifundefined{keywa}{}{\expandafter\MakeUppercase\keywa}%
        \@ifundefined{keywb}{}{, \keywb}%
        \@ifundefined{keywc}{}{, \keywc}%
        \@ifundefined{keywd}{}{, \keywd}%
        \@ifundefined{keywe}{}{, \keywe}%
        \@ifundefined{keywf}{}{, \keywf}%
        \@ifundefined{keywg}{}{, \keywg}%
        \@ifundefined{keywh}{}{, \keywh}%
        \@ifundefined{keywi}{}{, \keywi}%
        \@ifundefined{keywj}{}{, \keywj}%
}

%==============================================================================
% Redefinição da data (para ter mês e ano separados)
%==============================================================================
\renewcommand{\date}[2]{
        \gdef\@publmonth{#1}
        \gdef\@publyear{#2}
}
\date{\monthname}{\number\year}


%==============================================================================
% Definição da capa, folha de rosto e folha de aprovação.
% Os dados específicos do tipo de trabalho (dissertação, tese, etc.) são
% informados definindo-se o comando \titlepagespecificinfo. Veja os
% arquivos .def incluídos com a classe.
%==============================================================================
\newlength{\titlepagewinwidth}\setlength{\titlepagewinwidth}{95mm}
\newlength{\titlepagewinheight}\setlength{\titlepagewinheight}{64mm}
\newlength{\titlepagewinvoffset}\setlength{\titlepagewinvoffset}{112mm}
\newcommand{\location}[2]{
        \gdef\@city{#1}
        \gdef\@state{#2}
}
\location{Porto~Alegre}{RS}
\renewcommand{\maketitle}{%
	\@makecover
        \begin{titlepage}
                \thetitlepage
        \end{titlepage}%
	\if@openright\cleardoublepage\setcounter{page}{3}\else\clearpage\setcounter{page}{2}\fi%
%	\clearpage
%        \setcounter{page}{2}%
%        \@ifundefined{@cipspecificinfo}{}{\@makecippage}

        \@ifundefined{@approvalspecificinfo}{}{\@makeapprovalpage}

}
\newcommand{\thetitlepage}{%
        \noindent\@titlepagetopbox
        \vfill
        \@ifundefined{titlepagespecificinfo}{}{
                \hspace*{\fill}\makebox[.55\textwidth][l]{%
                        \parbox[b]{8.5cm}{\titlepagespecificinfo}%
                }
                \vfill
        }
        \@ifundefined{@advisor}{}{
                \hspace*{\fill}\parbox[b]{\@advisorwidth}{%
                        \titlepageadvisorinfo
                }
                \vfill
        }
        \centerline{\@city}
	\centerline{\@publyear}
}
\def\@titlepagetopbox{%
        \parbox[t]{\textwidth}{%
                \centering%
%                \vbox to \titlepagewinvoffset{%
			\@winbox
%                }
        }
}
\def\@winbox{%
%       \fbox{
        \parbox[b]{\titlepagewinwidth}{%
        \centering\vbox to \titlepagewinheight{%
                \vspace*{\fill}
                \MakeUppercase{\textbf{\@authora}}\\
                \@ifundefined{@authorb}{}{\MakeUppercase{\textbf{\@authorb}}\\}%
                \@ifundefined{@authorc}{}{\MakeUppercase{\textbf{\@authorc}}\\}%
                \@ifundefined{@authord}{}{\MakeUppercase{\textbf{\@authord}}\\}%
                \vspace{\stretch{4}}
                {\large\MakeUppercase{\textbf{\@title}}\par}
                \vfill
        }}%
%       }%
}
\newcommand{\course}[1]{\gdef\@course{#1}}

\newcommand{\titlepagetopinfo}{%
        \MakeUppercase{\ufrgs}\\
	\MakeUppercase{\ee}\\
        \@ifundefined{@dept}{}{\MakeUppercase{\csname \@dept\endcsname}\\}%
        \@ifundefined{@course}{}{\MakeUppercase{\@course}}
}
\newcommand{\titlepageadvisorinfo}{%
        \@ifundefined{@advisor}{}{\vfill\MakeUppercase{\advisorname}: \@advisor}
        \@ifundefined{@coadvisor}{}{\\[6ex]\MakeUppercase{\coadvisorname}: \@coadvisor}
}

\newcommand{\@makecover}{%
        \begin{titlepage}
                \@thecover
        \end{titlepage}%
	\if@openright\cleardoublepage\else\clearpage\fi%
}
\newcommand{\@thecover}{%
        \noindent\@covertopbox
        \vfill
        \centerline{\@city}
	\centerline{\@publyear}
}
\def\@covertopbox{%
        \parbox[t]{\textwidth}{%
                \centering%
                \vbox to \titlepagewinvoffset{%
                        \titlepagetopinfo\vfill\@winbox
                }
        }
}

\newcommand{\@makeapprovalpage}{%
        \begin{center}
                \@theapprovalpage
        \end{center}%
	\if@openright\cleardoublepage\else\clearpage\fi%
}
\newcommand{\@theapprovalpage}{%
        \noindent\@titlepagetopbox
        \vfill
        \@ifundefined{@approvalspecificinfo}{}{
                \hspace*{\fill}\makebox[.55\textwidth][l]{%
                        \parbox[b]{8.5cm}{\@approvalspecificinfo}%
                }
                \vfill
        }
        \@ifundefined{@advisor}{}{
                \hspace*{\fill}\parbox[b]{.70\textwidth}{%
                        \@approvalpageadvisorinfo
                }
                \vfill
        }
	\@ifundefined{@examinera}{}{
		\hspace*{\fill}\parbox[b]{1.0\textwidth}{\examinername:}
		\vfill
                \hspace*{\fill}\parbox[b]{.90\textwidth}{%
                        \@approvalpageexaminerinfo
                }
                \vfill
        }
        \@ifundefined{coord}{}{
                \hspace*{\fill}\parbox[b]{.70\textwidth}{%
                        \@approvalpagecoordinfo
                }
                \vfill
        }
        \vfill
        \hfill\@city, \delet@date.
}
\newcommand{\@approvalpageadvisorinfo}{%
        \@ifundefined{@advisor}{}{%
		\vfill
		\advisorname: \hrulefill\\
		\@advisor, \@advisorinstitution\\
		\@advisortitleinfo 
	}
}
\newcommand{\@approvalpageexaminerinfo}{%
		\vfill
		\@examinera, \@examinerainstitution\\
		\@examineratitleinfo\\

                \@ifundefined{@examinerb}{}{%
			\@examinerb, \@examinerbinstitution\\
			\@examinerbtitleinfo\\

		}%
                \@ifundefined{@examinerc}{}{%
			\@examinerc, \@examinercinstitution\\
			\@examinerctitleinfo\\

		}%
                \@ifundefined{@examinerd}{}{%
			\@examinerd, \@examinerdinstitution\\
			\@examinerdtitleinfo\\

		}%
                \@ifundefined{@examinere}{}{%
			\@examinere, \@examinereinstitution\\
			\@examineretitleinfo\\
		}%
}
\newcommand{\@approvalpagecoordinfo}{%
        \@ifundefined{coord}{}{%
		\vfill
		\coordname: \hrulefill\\
		\coord 
	}
}

%==============================================================================
% Catalogação na fonte (CIP) e nominata
%==============================================================================
\newcommand{\@makecippage}{%
        \clearpage%
        \begin{center}
                \textbf{\cipcaption}\\[2ex]
                \deletexu@cipwindow
        \end{center}
        \vfill
        \nominata
}
% definição do quadro
\def\deletexu@cipwindow{%
        \framebox[120mm]{%
        \begin{minipage}{110mm}%
                \vspace*{0.2ex}
                \setlength{\parindent}{1.5em}%
                \setlength{\parskip}{1.3ex}%
                \noindent\@authoralast, \@authorafirst\par
                \@title\ / \@authora.\ -- \@city: PPGEE da~UFRGS, \@publyear.\par
                \pageref{LastPage}~f.: il.\par
                \@cipspecificinfo\ -- \ufrgs. \@course, \@city, BR--\@state, \@publyear. \advisorname: \@advisorfirst\ \@advisorlast\@ifundefined{@coadvisor}{}{; \coadvisorname: \@coadvisorfirst\ \@coadvisorlast}.\par
                \@cipkw I.~\@advisorlast, \@advisorfirst. \@ifundefined{@coadvisorfirst}{II.~T\'{\i}tulo.}{II.~\@coadvisorlast, \@coadvisorfirst. III.~T\'{\i}tulo.}\\
                \vspace{0.2ex}
        \end{minipage}%
        }%
}
\newcommand{\nominataReit}{Prof\textsuperscript{a}.~Dr.~Jos{\'e} Carlos Ferraz Hennemann}
\newcommand{\nominataReitname}{Reitor}
\newcommand{\nominataPRE}{Prof.~Dr.~Pedro Cezar Dutra Fonseca}
\newcommand{\nominataPREname}{Pr{\'o}-Reitor de Ensino}
\newcommand{\nominataPRAPG}{Prof\textsuperscript{a}.~Dr\textsuperscript{a}.~Valqu\'{\i}ria Linck Bassani}
\newcommand{\nominataPRAPGname}{Pr{\'o}-Reitora de P{\'o}s-Gradua{\c{c}}{\~a}o}
\newcommand{\nominataDir}{Prof.~Dr.~Renato Machado de Brito}
\newcommand{\nominataDirname}{Diretor da Escola de Engenharia}
\newcommand{\nominataCoord}{Prof.~Dr.~Carlos Eduardo Pereira}
\newcommand{\nominataCoordname}{Coordenador do PPGEE}
\newcommand{\nominataBibchefe}{June Magda Rosa Schamberg}
\newcommand{\nominataBibchefename}{Bibliotec{\'a}ria-chefe da Escola de Engenharia}
\newcommand{\nominataChefeDELET}{Prof.~Dr.~Roberto Petry Homrich}
\newcommand{\nominataChefeDELETname}{Chefe do \delet}
\newcommand{\nominata}{
        \MakeUppercase{\ufrgs}\\
        \nominataReitname: \nominataReit\\
        \nominataPREname: \nominataPRE\\
        \nominataPRAPGname: \nominataPRAPG\\
        \nominataDirname: \nominataDir\\
        \nominataCoordname: \nominataCoord\\
        \nominataBibchefename: \nominataBibchefe
}

%==============================================================================
% Ajuste do sumário (nada indentado) 
%==============================================================================
\newlength{\deletex@chapnumindent}\setlength{\deletex@chapnumindent}{1.8em}
\renewcommand*{\l@chapter}[2]{
        \addpenalty{-\@highpenalty}
        \vskip -1ex \@plus\p@
        \setlength\@tempdima{1.5em}
        \begingroup
        \parindent \z@ \rightskip \@pnumwidth
        \parfillskip -\@pnumwidth
        \leavevmode\bfseries\sffamily
        \advance\leftskip\@tempdima
        \hskip -\leftskip
%      esta é a definicao original da classe report:
%      #1\nobreak\hfil \nobreak\hb@xt@\@pnumwidth{\hss #2}\par
        \@dottedtocline{0}{0em}{\deletex@chapnumindent}{\uppercase{#1}}{#2}
%        \@dottedtocline{0}{0em}{\deletex@chapnumindent}{\MakeUppercase{#1}}{#2}
        \penalty\@highpenalty
        \endgroup
}
\renewcommand*{\l@section}[2]
              {\@dottedtocline{1}{0em}{2.8em}{\textbf{#1}}{\hss #2}}
\renewcommand*{\l@subsection}{\@dottedtocline{2}{0em}{3.7em}}
\newif\if@aftertoc
\@aftertocfalse
\renewcommand{\tableofcontents}{%
        \chapter*{\contentsname}
        \@starttoc{toc}\@aftertoctrue%
}

%==============================================================================
% Definição das listas de abreviaturas e siglas e de símbolos
%==============================================================================
\newenvironment{listofabbrv}[1]{
        \chapter*{\listabbrvname}
        \begin{list}{\textbf{??}}{
                \settowidth{\labelwidth}{#1}
                \setlength{\labelsep}{1em}
                \setlength{\itemindent}{0mm}
                \setlength{\leftmargin}{\labelwidth}
                \addtolength{\leftmargin}{\labelsep}
                \setlength{\rightmargin}{0mm}
                \setlength{\itemsep}{.1\baselineskip}
                \renewcommand{\makelabel}[1]{\makebox[\labelwidth][l]{##1}}
        }
}{
        \end{list}
}
\newenvironment{listofsymbols}[1]{
        \chapter*{\listsymbolname}
        \begin{list}{\textbf{??}}{
                \settowidth{\labelwidth}{#1}
                \setlength{\labelsep}{1em}
                \setlength{\itemindent}{0mm}
                \setlength{\leftmargin}{\labelwidth}
                \addtolength{\leftmargin}{\labelsep}
                \setlength{\rightmargin}{0mm}
                \setlength{\itemsep}{.1\baselineskip}
                \renewcommand{\makelabel}[1]{\makebox[\labelwidth][l]{##1}}
        }
}{
        \end{list}
}

%==============================================================================
% Resumo (abstract) e Abstract (englishabstract)
%==============================================================================
\renewenvironment{abstract}{%
        \chapter*{\abstractname}
}{%
        \par\vspace{2ex} %\vfill
        \noindent\textbf{\keywordsname: \@abstractkw.}
}
\newenvironment{englishabstract}[1]{%
        \if@openright\cleardoublepage\else\clearpage\fi%
        \gdef\@englishkeywords{#1}%
        \iflanguage{english}{
                \begin{otherlanguage}{brazilian}
        }{
                \begin{otherlanguage}{english}
        }
                \chapter*{\abstractname}
}{
        \par\vspace{2ex} %\vfill
        \noindent\textbf{\keywordsname: \@englishkeywords.}
        \end{otherlanguage}
}

%==============================================================================
% Formatação dos capítulos.
% Os comandos \@chapter e \@schapter são redefinidos para que a
% numeração de páginas seja correta (deve aparecer somente a partir do
% primeiro capítulo do texto, mas não nas páginas iniciais de
% capítulos).
% Os comandos \@makechapterhead e \@makeschapterhead fazem a
% formatação dos títulos propriamente ditos.
%==============================================================================
\newcommand{\deletex@chapterfont}{\@setfontsize\Large{16}{20}}
\def\@chapapp{}
\def\@chapter[#1]#2{%
        \refstepcounter{chapter}%
        \addcontentsline{toc}{chapter}{\protect\numberline{\@chapapp\thechapter}#1}%
        \addtocontents{lof}{\protect\addvspace{10\p@}}%
        \addtocontents{lot}{\protect\addvspace{10\p@}}%
        \pagestyle{deletex}\let\ps@plain\ps@deletex%
        \@makechapterhead{#2}\@afterheading
}
\def\@makechapterhead#1{%
        \vspace*{90pt}
        \reset@font\noindent{\deletex@chapterfont\sffamily\textbf{\@chapapp\thechapter\hspace{1em}\MakeUppercase{#1}}\par}
        \addvspace{42pt}
}
\def\@schapter#1{%
        \if@aftertoc\addcontentsline{toc}{chapter}{#1}\fi%
        \addtocontents{lof}{\protect\addvspace{10\p@}}%
        \addtocontents{lot}{\protect\addvspace{10\p@}}%
        \@makeschapterhead{#1}\@afterheading
}
\def\@makeschapterhead#1{%
        \vspace*{90pt}
        \reset@font\centerline{\deletex@chapterfont\sffamily\textbf{\MakeUppercase{#1}}}
        \addvspace{42pt}
}
\renewcommand{\appendix}[1][\appendixname]{%
        \setcounter{chapter}{0}%
        \renewcommand{\thechapter}{\Alph{chapter}}%
        \setcounter{tocdepth}{0}%
        \@captionlistingfalse%
        \gdef\@chapapp{\MakeUppercase{#1}\space}%
        \addtocontents{toc}{%
          \settowidth{\deletex@chapnumindent}{{\sffamily\textbf{\@chapapp}}}
          \addtolength{\deletex@chapnumindent}{1.8em}
        }%
}
\newcommand{\annex}{\appendix[\annexname]}

%==============================================================================
% Formatação das seções
%==============================================================================
\setcounter{secnumdepth}{4}
\renewcommand{\section}{%
        \@startsection{section}{1}{0pt}{18pt}{9pt}{\reset@font\large\bfseries}
}
\renewcommand{\subsection}{%
        \@startsection{subsection}{2}{0pt}{12pt}{6pt}{\reset@font\bfseries}
}
\renewcommand{\subsubsection}{%
        \@startsection{subsubsection}{3}{0pt}{12pt}{6pt}{\reset@font\itshape}
}
\renewcommand{\paragraph}{%
        \@startsection{paragraph}{4}{0pt}{12pt}{6pt}{\reset@font}
}
\renewcommand{\subparagraph}{%
        \@startsection{subparagraph}{5}{0pt}{1ex}{-1em}{\reset@font\bfseries}
}

%==============================================================================
% Redefinição do conteúdo das listas de figuras e tabelas
%==============================================================================
\newif\if@captionlisting\@captionlistingtrue
\long\def\@caption#1[#2]#3{%
  \par
  \if@captionlisting%
    \addcontentsline{\csname ext@#1\endcsname}{#1}%
        {\protect\numberline{\csname fnum@#1\endcsname:}{\ignorespaces #2}}%
  \fi%
  \begingroup
    \@parboxrestore
    \if@minipage
      \@setminipage
    \fi
    \normalsize
    \@makecaption{\csname fnum@#1\endcsname}{\ignorespaces #3}\par
  \endgroup%
}
\renewcommand*{\l@figure}{\@dottedtocline{1}{0em}{5.5em}}
\let\l@table\l@figure

%==============================================================================
% Redefinição do formato de citações longas
%==============================================================================
\renewenvironment{quote}{%
        \small%
        \begin{list}{}{\setlength{\leftmargin}{40mm}\item\relax}
}{%
        \end{list}
}

%==============================================================================
% Ajuste do formato das citações e referências
%==============================================================================
\setlength{\bibhang}{0pt}
\let\cite\citep

%==============================================================================
% Ajuste do formato das numeracoes das figuras, tabelas e equações
%==============================================================================
\if@relnum

\else

\RequirePackage{remreset}

\@removefromreset{equation}{chapter}
\renewcommand{\theequation}{\@arabic\c@equation}

\@removefromreset{figure}{chapter}
\renewcommand{\thefigure}{\@arabic\c@figure}

\@removefromreset{table}{chapter}
\renewcommand{\thetable}{\@arabic\c@table}

\fi
