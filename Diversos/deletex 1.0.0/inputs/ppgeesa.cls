%==============================================================================
% ppgeesa - Seminario de Andamento do PPGEE
% Copyright (C) 2004 Walter Fetter Lages <w.fetter@ieee.org>
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

\NeedsTeXFormat{LaTeX2e}
\ProvidesClass{ppgeesa}[2004/11/29 v0.4.0 Pacote DeLeTeX - ppgeesa.cls - Seminario de Andamento do PPGEE]

\DeclareOption{english}{\@englishtrue\OptionNotUsed}\newif\if@english
\DeclareOption{twoside}{\PassOptionsToClass{twoside}{IEEETran}}
%\DeclareOption*{\PassOptionsToClass{\CurrentOption}{IEEETran}}
\ProcessOptions\relax

\LoadClass[10pt,final,journal,a4paper]{IEEEtran}
\RequirePackage[english,brazilian]{babel}
\RequirePackage{cite}
\input{deletex_pt_BR.ldf}
\input{deletex_en.ldf}
\if@english
        \selectlanguage{english}
        \AtBeginDocument{\selectlanguage{english}}
\fi
\RequirePackage{deletexdefs}

% Redefines abstract and keywords to show correct names in portuguese
%
\def\abstract{\normalfont%
    \if@twocolumn%
      \@IEEEabskeysecsize\bfseries\textit{\abstractname}---\,%
    \else%
      \begin{center}\vspace{-1.78ex}\@IEEEabskeysecsize\textbf{\abstractname}\end{center}\quotation\@IEEEabskeysecsize%
    \fi\@IEEEgobbleleadPARNLSP}
% V1.6 IEEE wants only 1 pica from end of abstract to introduction heading when in 
% conference mode (the heading already has this much above it)
\def\endabstract{\relax\if@confmode\vspace{0ex}\else\vspace{1.34ex}\fi\par\if@twocolumn\else\endquotation\fi%
    \normalfont\normalsize}


\def\keywords{\normalfont%
    \if@twocolumn%
      \@IEEEabskeysecsize\bfseries\textit{\keywordsname}---\,\relax%
    \else%
      \begin{center}\@IEEEabskeysecsize\bfseries \keywordsname\end{center}\quotation\@IEEEabskeysecsize%
    \fi\@IEEEgobbleleadPARNLSP}
\def\endkeywords{\relax\if@technote\vspace{1.34ex}\else\vspace{0.67ex}\fi%
    \par\if@twocolumn\else\endquotation\fi%
    \normalsize\normalfont}

% redefines tablename
%
\def\fnum@table{\MakeUppercase{\tablename}~\thetable}

% redefines bibname
%
\def\thebibliography#1{\section*{\bibname}%
    \addcontentsline{toc}{section}{\bibname}%
    % V1.6 add some rubber space here and provide a command trigger
    \footnotesize \vskip 0.3\baselineskip plus 0.1\baselineskip minus 0.1\baselineskip%
    \list{\@biblabel{\@arabic\c@enumiv}}%
    {\settowidth\labelwidth{\@biblabel{#1}}%
    \leftmargin\labelwidth
    \advance\leftmargin\labelsep\relax
    \itemsep 0pt plus .5pt\relax%
    \usecounter{enumiv}%
    \let\p@enumiv\@empty
    \renewcommand\theenumiv{\@arabic\c@enumiv}}%
    \let\@IEEElatexbibitem\bibitem%
    \def\bibitem{\@IEEEbibitemprefix\@IEEElatexbibitem}%
\def\newblock{\hskip .11em plus .33em minus .07em}%
% originally:
%   \sloppy\clubpenalty4000\widowpenalty4000%
% by adding the \interlinepenalty here, we make it more
% difficult, but not impossible, for LaTeX to break within a reference.
% IEEE almost never breaks a reference (but they do it more often with
% technotes). You may get an underfull vbox warning around the bibliography, 
% but the final result will be much more like what IEEE will publish. 
% MDS 11/2000
\if@technote\sloppy\clubpenalty4000\widowpenalty4000\interlinepenalty100%
\else\sloppy\clubpenalty4000\widowpenalty4000\interlinepenalty500\fi%
    \sfcode`\.=1000\relax}

\centerfigcaptionstrue
