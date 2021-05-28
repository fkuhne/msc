%
%
% LaTeX2e
%
% SBAI - Simposio Brasileiro de Automacao Inteligente
%
% $Id: sbai.cls,v 1.4 2003/07/25 19:07:30 poli Exp $
%
% Classe para apresentacao de trabalhos no Simposio
%
% por Marco A. Poli <marco.poli<AT>poli.usp.br>
% Departamento de Engenharia Mecatr\^onica
% Escola Polit\'ecnica
% Universidade de S\~ao Paulo
%
% Baseado no sbatex.cls versao 1.0 alpha por Mauricio C. de Oliveira
% (mcdeoliveira<AT>ieee.org)
%
%


% Instrucoes:
%
%
% Eh necessario o uso do estilo hardvard.sty e o estilo BiBTeX dcu.bst para
% bibliografias que podem ser obtidos em
% ftp://ftp.ctan.org/tex-archive/macros/latex/contrib/supported/harvard
%
% Juntamente com o pacote fancyhdr, que pode ser obtido em:
% ftp://ftp.ctan.org/tex-archive/macros/latex/contrib/supported/fancyhdr
%
% tambem dos pacotes ifthen
% ftp://ftp.ctan.org/tex-archive/macros/latex/base/ifthen.ins e .dtx
%
% calc
% ftp://ftp.ctan.org/tex-archive/macros/latex/required/tools/calc.dtx
%
% a fonte times
% ftp://ftp.ctan.org/tex-archive/fonts/psfonts/corelpak/times
%
% o pacote setspace
% ftp://ftp.ctan.org/tex-archive/macros/latex/contrib/supported/setspace
%
% babel
% ftp://ftp.ctan.org/tex-archive/language/babel
%
% theorem e
% ftp://ftp.ctan.org/tex-archive/macros/latex/required/tools/theorem.dtx
%
% geometry
% ftp://ftp.ctan.org/tex-archive/macros/latex/contrib/supported/geometry
%
% Muitas intalacoes do LaTeX ja possuem grande parte desses pacotes instalados,
% portanto, antes de baixa-los, verifique se ja nao os tem.
%
%
% O pacote requer 4 variaveis inseridas no arquivo fonte:
% Variaveis necessarias para o SBAI
% \edicaosbai{}            % em numerais romanos letras maiusculas
% \messbai{}               % mes em que trasncorrera o simposio, minusculas
% \anosbai{}               % ano no formato AAAA
% \localsbai{}             % Cidade onde ocorrera o simposio, primeira letra
%                          %        em letra maiuscula
% /SBAI
%
% Por exemplo:
% \edicaosbai{VI}          % VI SBAI
% \messbai{setembro}       % ocorrera no mes de setembro
% \anosbai{2003}           % do ano de 2003
% \localsbai{Bauru}        % na cidade de Bauru
% /SBAI
%
% Sem essas 4 veriaveis definidas, o latex ira gerar uma serie warnings, e o
% dvi gerado nao estara conforme o padrao necessario.
%
% O arquivo artigo-sbai.tex fornece um exemplo de como o arquivo fonte deve
% ser para que essa classe gere o resultado esperado. Em \documentclass, as
% seguintes opcoes devem ser definidas:
%
% \documentclass[journal,harvard]{sbai} juntamente com as opcoes de lingua
% desejadas, por exemplo: \documentclass[brazil,journal,harvard]{sbai}
%
% Para compilar este exemplo use a seqüência de comandos:
%
%     latex artigo-sbai
%     bibtex artigo-sbai
%     latex artigo-sbai
%     latex artigo-sbai
%
% Para gerar um arquivo Postscript (.ps):
%
%     dvips -t a4 artigo-sbai
%
% Recomenda-se o uso da versao dvips 5.86e ou superior.
%
%  esse comando, no entando, em algumas instalacoes, ira produzir arquivos
%  nos quais algumas letras estarao erradas. Notadamente, a combinacao ``fl''
%  sera substituida pelo simbolo da libra esterlina.
%
%  Nesses casos, utilize o comando:
%
%       dvips -ta4 -G0 artigo-sbai
%
%
% Para gerar um arquivo Portable Document Format (.pdf):
%
%     dvips -Ppdf -ta4 -G0 artigo-sbai
%     ps2pdf -dMaxSubsetPct=100 -dSubsetFonts=true -dEmbedAllFonts=true \
%            -dCompatibilityLevel=1.2 -sPAPERSIZE=a4 artigo-sbai.ps
%
% Faz-se necessaria a utilizacao da versao 6.50 ou superior do
% GhostScript para a correta operacao do ps2pdf.
%

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{sbai}
[2003/07/25 LaTeX2e class for SBAI - 1.0]

% process submission option
\RequirePackage{ifthen,calc}
\newboolean{isSubmission}
\setboolean{isSubmission}{false}

\DeclareOption{submission}{
  \setboolean{isSubmission}{true}
}

% Set up journal
\DeclareOption{journal}{

  \def\anosbai#1{\gdef\@anosbai{#1}}
  \def\@anosbai{ANO\@latex@warning@no@line{No \noexpand\messbai given}}

  \def\messbai#1{\gdef\@messbai{#1}}
  \def\@messbai{MES\@latex@warning@no@line{No \noexpand\messbai given}}

  \def\localsbai#1{\gdef\@localsbai{#1}}
  \def\@localsbai{CIDADE\@latex@warning@no@line{No \noexpand\localsbai given}}

  \def\edicaosbai#1{\gdef\@edicaosbai{#1}}
  \def\@edicaosbai{EDI\c{C}\~{A}O\@latex@warning@no@line{No \noexpand\edicaosbai given}}


  \newcommand{\aceito}[1]{
    \renewcommand{\@makefnmark}{}
    \footnotetext{{\sffamily #1}}
    \renewcommand{\@makefnmark}{\ensuremath{^{\@thefnmark}}}
    \setcounter{footnote}{0}
  }
  \newcommand{\pagina}[1]{\setcounter{page}{#1}}

  \ifthenelse{\boolean{isSubmission}}{
    % pass options to class article
    \PassOptionsToClass{11pt,onecolumn,oneside}{article}
  }{
    % pass options to class article
    \PassOptionsToClass{10pt,twocolumn,twoside}{article}
  }
  \AtEndOfPackage{

    % section layout
    \renewcommand\section{\@startsection{section}{1}
                          {0pt}
                          {-6pt}
                          {1pt}
                         {\large\sffamily\bfseries\MakeUppercase}}
    \renewcommand\subsection{\@startsection{subsection}{2}
                             {0pt}
                             {-6pt}
                             {1pt}
                             {\large\sffamily\bfseries}}
    \renewcommand\subsubsection{\@startsection{subsubsection}{3}
                                {0pt}
                                {-6pt}
                                {1pt}
                                {\normalsize\sffamily\bfseries}}

    % authors and address
    \newcounter{author}
    \newcounter{tmpauthor}

    \newcommand\listofauthors{\@starttoc{loa}}
    \newcommand{\l@authors}[2]{
      \begin{minipage}[t]{0.49\textwidth}
        \begin{center}
          \begin{tabular}{c}
            #1
          \end{tabular}
        \end{center}
      \end{minipage}
    }

    \newcommand{\newauthor}[3][-1]{
      \ifthenelse{\equal{#1}{-1}}
        { \stepcounter{author} }
        { \setcounter{tmpauthor}{\theauthor}
          \setcounter{author}{#1} }
      \addcontentsline{loa}{authors}
        { {\large\bfseries #2$^\fnsymbol{author}$} \\ {\ttfamily #3} }
      \ifthenelse{\equal{#1}{-1}}
        {}
        { \setcounter{author}{\thetmpauthor} }
    }

    \newcommand{\newaddress}[1]{
      \stepcounter{address}
      \addcontentsline{los}{addresses}
        { {$^\fnsymbol{address}$#1} }
    }

    \let\author\newauthor
    \let\address\newaddress

    % maketitle
    \def\@maketitle{%
      \newpage
      \begin{center}%
        \hbox{\rule{\textwidth}{6pt}}
        \vskip 1pt
        \hbox{\rule{\textwidth}{0.4pt}}
        \vskip 3mm
        {\Large\sffamily\bfseries\MakeUppercase\@title \par}%
        \vskip 12mm
        \lineskip .5em
        {\centering{\listofauthors} \par}
        \vskip 6pt
        {\centering{\listofaddresses} \par}
        \vskip 8mm
        \hbox{\rule{\textwidth}{6pt}}
        \vskip 1pt
        \hbox{\rule{\textwidth}{0.4pt}}
        \vskip 30pt
      \end{center}
    }

    % set font = times on journal style
    \RequirePackage{times}

    \ifthenelse{\boolean{isSubmission}}{
      % set page dimensions
      \geometry{margin=10mm,nohead,nofoot,twosideshift=0pt,top=10mm,bottom=10mm}
%      \geometry{margin=2cm,nohead,nofoot}

      % load set space
      \RequirePackage{setspace}
    }{
      % set page dimensions
      %\geometry{margin=17.5mm,twosideshift=2.5mm,top=50mm,bottom=11mm,footskip=9mm,nohead}
%      \geometry{margin=17.5mm,twosideshift=2.5mm,top=45mm,headsep=0pt,headheight=5mm,bottom=11mm,footskip=9mm,}

%%       \geometry{margin=17.5mm,twosideshift=0pt,top=10mm,headsep=0pt,headheight=5mm,bottom=11mm,footskip=9mm}

       \geometry{margin=10mm,nohead,nofoot,twosideshift=0pt,top=10mm,headsep=0pt,bottom=10mm,footskip=10mm}


% Marcacao desabilidata
      % include marker
%       \special{!
%         userdict begin
%           /mm {72 mul 25.4 div} def % Convert inches->points (1/72 inch)
%           /bop-hook {
%             newpath                 % Start a new path
%             0 mm 267 mm moveto      %
%             30 mm 267 mm lineto     %
%             180 mm 267 mm moveto    %
%             210 mm 267 mm lineto    %
%             0.1 setlinewidth        % set line width
%             stroke                  % Draw the outer lines
%           } def end
%       }
    }

    % header and footer
    \RequirePackage{fancyhdr}
    \renewcommand\headrulewidth{0pt}
    \ifthenelse{\boolean{isSubmission}}{
      \def\currentfooter{Artigo proposto \`{a} Revista Controle \& Automa\c{c}\~{a}o em \today}
    }{
%      \def\currentfooter{\begin{flushleft} \@edicaosbai\ SBAI / II IEEE LARS. \@localsbai, \@messbai\ de \@anosbai \end{flushleft}}
      \def\currentfooter{ \@edicaosbai\ SBAI / II IEEE LARS. \@localsbai, \@messbai\ de \@anosbai }
    }
    \cfoot{} \rhead{} \lhead{}
    \lfoot[\footnotesize \sffamily\bfseries \currentfooter \hskip 8mm]{\footnotesize \sffamily\bfseries \currentfooter \hskip 8mm}
    \rfoot[\footnotesize \sffamily\bfseries \thepage ]{\footnotesize \sffamily\bfseries \thepage}
%   \rfoot[\footnotesize \sffamily\bfseries \currentfooter \hskip 8mm]{\footnotesize \sffamily\bfseries \currentfooter \hskip 8mm}


%     \lfoot[\footnotesize \sffamily\bfseries \thepage \hskip 3mm \currentfooter]{}
%     \rfoot[]{\footnotesize \sffamily\bfseries \currentfooter \hskip 3mm \thepage}
%       \def\currentfooter{Revista Controle \&
%       Automa\c{c}\~{a}o/Vol.\@volume~no.\@numero/\@mes\ \@ano}


    % set page style
    \pagestyle{fancy}

    % page layout parameters
    \setlength\parindent{0pt}
    \setlength\parskip{9pt}
%    \setlength\columnsep{5mm}
    \setlength\columnsep{10mm}

    % Set theorem body font to roman
    \theorembodyfont{\rmfamily}

    %% Keywords
    \def\keywords#1{
      \par
      {\small \sffamily \textbf{\MakeUppercase{\keywordlabel}}}: #1
      \par
    }
  }


  % turn on double space
  \ifthenelse{\boolean{isSubmission}}{
    \AtBeginDocument{\begin{doublespace}}
    \AtEndDocument{\end{doublespace}}
  }{}

}

% set up conference
\DeclareOption{conference}{

  % ignore submission flag
  \setboolean{isSubmission}{false}

  % pass options to class article
  \PassOptionsToClass{10pt,twocolumn,oneside}{article}
  \AtEndOfPackage{

    % section layout
    \renewcommand\section{\@startsection{section}{1}
                         {\z@}
                         {-3.5ex plus -1ex minus -.2ex}
                         {2.3ex plus .2ex}
                         {\centering\bfseries}}
    \renewcommand\subsection{\@startsection{subsection}{2}
                            {\z@}
                            {-3.25ex plus -1ex minus -.2ex}
                            {1.5ex plus .2ex}
                            {\itshape}}

    % authors and address
    \newcounter{author}
    \newcounter{tmpauthor}

    \def\@eebuffer{}
    \def\addemail #1{\expandafter\def\expandafter\@eebuffer\expandafter
                     {\@eebuffer #1\relax}}

    \newcommand\listofauthors{\@starttoc{loa}}
    \newcommand{\l@authors}[2]{#1}

    \newcommand{\newauthor}[3][-1]{
      \setcounter{tmpauthor}{\theauthor}
      \ifthenelse{\equal{#1}{-1}}
        { \stepcounter{author} }
        { \setcounter{author}{#1} }
      \ifthenelse{\value{tmpauthor} = 0}
        { }
        { \addcontentsline{loa}{authors}{\hspace{-2ex}~,}
          \addemail{, } }
      \addemail{\ttfamily{#3}}
      \addcontentsline{loa}{authors}{\scshape{#2}$^\fnsymbol{author}$}
      \ifthenelse{\equal{#1}{-1}}
        {}
        { \setcounter{author}{\thetmpauthor} }
    }

    \newcommand{\newaddress}[1]{
      \stepcounter{address}
      \addcontentsline{los}{addresses}
        { {\normalsize\itshape{$^\fnsymbol{address}$#1}} }
    }

    \let\author\newauthor
    \let\address\newaddress

    % maketitle
    \def\@maketitle{%
      \newpage
      \begin{center}
         {\bf \MakeUppercase \@title \par}
         \vskip 2em                      % Vertical space after title.
         \lineskip .5em
         {\centering{\listofauthors} \par}
         \vskip 6pt
         {\centering{\listofaddresses} \par}
         \vskip 3pt
         {\centering{
            \ifthenelse{\value{tmpauthor} = 0}
              { Email: }
              { Emails: }
            \@eebuffer} \par}
      \end{center}
      \vskip 1.5em \par
    }

    % paragraph with argument to make a runaway title.
    \def\namedpar#1{\paragraph{\footnotesize\bfseries #1---}}

    % redifine abstract
    \renewenvironment{abstract}{
      \begin{center}
        \begin{minipage}{5.75truein}
          \footnotesize
          \namedpar{\abstractname}
    }{
        \end{minipage}
      \end{center}
      \vskip 1ex \par
    }

    % header and footer
    %\RequirePackage{fancyhdr}
    %\renewcommand\headrulewidth{0pt}
    %\cfoot{} \lfoot{} \rfoot{} \rhead{} \lhead{}

    % Set page style
    \pagestyle{empty}

    % page layout parameters
    \setlength{\parindent}{0.25in}
%    \setlength{\columnsep}{0.3in}
    \setlength{\columnsep}{10mm}

    % list parameters
    \setlength{\topsep}{1ex}
    \setlength{\partopsep}{0in}
    \setlength{\parsep}{0in}
    \setlength{\itemsep}{0in}

    % set page dimensions
    \geometry{width=6.27in,height=10in,nohead,nofoot}

    %% Keywords
    \def\keywords#1{
      \renewcommand{\abstractname}{\keywordlabel}
      \begin{abstract}
        #1
      \end{abstract}
    }
  }
}

\DeclareOption{harvard}{

  \AtEndOfPackage{

    % load package harvard
    \RequirePackage{harvard}

    % configure style
    \iflanguage{english}{}{
        \renewcommand{\HAR@and@agsm}{\&}
        \renewcommand{\HAR@and@dcu}{e}
        \renewcommand{\HAR@and@apsr}{e}
    }
    \citationmode{abbr}
    \citationstyle{dcu}
    \bibliographystyle{dcu}
  }
}

% Process all unknown options through article
\DeclareOption*{
  \PassOptionsToPackage{\CurrentOption}{babel}
  \PassOptionsToClass{\CurrentOption}{article}
}

% Do final as default
%\ifthenelse{\boolean{isSubmission}}
%{\ExecuteOptions{journal}}{}

% Process all options
\ProcessOptions

% Based on article.cls
\LoadClass[a4paper]{article}

% Load Packages

\RequirePackage{babel}
\RequirePackage{theorem}
\RequirePackage{geometry}
\RequirePackage{latexsym}

%% List of adresses
\newcommand\listofaddresses{\@starttoc{los}}
\newcommand{\l@addresses}[2]{#1 \par \vskip 6pt}

\newcounter{address}
\setcounter{address}{0}

%% Thanks
%\def\thanks#1{
%  \protected@xdef\@thanks{\@thanks\protect\footnotetext[\the\c@footnote]{#1}}
%}

%% Define maketitle command
\renewcommand\maketitle{
  \par
  \ifthenelse{\boolean{isSubmission}}{\begin{singlespace}}{}
  \begingroup
    \if@twocolumn
      \ifnum \col@number=\@ne
        \@maketitle
      \else
        \twocolumn[\@maketitle]%
      \fi
    \else
      \newpage
      \global\@topnum\z@   % Prevents figures from going at top of page.
      \@maketitle
    \fi
    %\thispagestyle{fancy}
    \renewcommand{\thefootnote}{\fnsymbol{footnote}}
    {\sffamily \@thanks}
    \renewcommand{\thefootnote}{\arabic{footnote}}
  \endgroup
  \ifthenelse{\boolean{isSubmission}}{\end{singlespace}}{}
}

% Multilanguage labels
\def\theoremlabel{\iflanguage{english}{Theorem}{Teorema}}
\def\corollarylabel{\iflanguage{english}{Corollary}{Corol\'ario}}
\def\lemmalabel{\iflanguage{english}{Lemma}{Lema}}
\def\prooflabel{\iflanguage{english}{Proof}{Prova}}
\def\keywordlabel{\iflanguage{english}{Keywords}{Palavras-chave}}

% Theorems
\newtheorem{theorem}{\theoremlabel}
\newtheorem{corollary}[theorem]{\corollarylabel}
\newtheorem{lemma}[theorem]{\lemmalabel}

\newenvironment{proof}
{ \begin{trivlist} \item[{ \bf \prooflabel:}] }
{ ~\hfill$\Box$ \end{trivlist} }


%%%%%%%%%%%%%%%%%%%%%%
% The logo of sbatex %
%%%%%%%%%%%%%%%%%%%%%%

\DeclareRobustCommand{\SBATeX}{{\small SBA}\kern-.05em\TeX}
