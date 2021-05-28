%==============================================================================
% deleticle - DELET article
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
\ProvidesClass{deleticle}[2004/11/29 v0.4.0 Pacote DeLeTeX - deleticle.cls - DELET Article]

\DeclareOption{english}{\@englishtrue\OptionNotUsed}\newif\if@english
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{article}}
\ProcessOptions\relax

\LoadClass[12pt,a4paper]{article}
\RequirePackage[english,brazilian]{babel}
\input{deletex_pt_BR.ldf}
\input{deletex_en.ldf}
\if@english
        \selectlanguage{english}
        \AtBeginDocument{\selectlanguage{english}}
\fi
\RequirePackage[T1]{fontenc}
\RequirePackage{times}
\RequirePackage{deletexdefs}

\newcommand{\course}[1]{\gdef\@course{#1}}

\def\@maketitle{%
        \begin{center}
                {\Large\ufrgs\par\ee\par\delet\par
		\@ifundefined{@course}{}{\@course}\par
		\vspace{2ex}
		\textbf{\@title}\par
                \vspace{2ex}
                \@author\par
		\vspace{2ex}
		\@date}
        \end{center}%
}
