%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% CUG 课程实验报告模板
% 作者：Yangmomo1978
% License: Creative Commons CC BY 4.0
% GitHub: https://github.com/Yangmomo1978/CUG_Report_Template
% Overleaf:
% 在SYSU_Latex_Template基础上修改而来， 原作者：NorthSecond
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SYSU 课程论文模板
% 作者：NorthSecond
% License: Creative Commons CC BY 4.0
% GitHub: https://github.com/NorthSecond/SYSU_Latex_Template
% Overleaf:
% 在UCAS_Latex_Template基础上修改而来， 原作者：jweihe
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%湖南大学课程论文模板
% 作者: Chenyang Zhang
% License: LaTeX Project Public License v1.3c
% 完整编译: XeLaTex -> BibTex -> XeLaTex -> XeLaTex
% GitHub项目地址：https://github.com/zcyeee/HNU_LaTeX_Template
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%UCAS课程论文模板
%%作者：jweihe
%%License：Creative Commons CC BY 4.0
%%GitHub：https://github.com/jweihe/UCAS_Latex_Template
%%Overleaf：https://www.overleaf.com/latex/templates/zhong-guo-ke-xue-yuan-da-xue-ke-cheng-lun-wen-mo-ban/nphpxhhqvnds

%%最后更改于2023/06/05
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%在"北京大学课程论文模板"基础上修改而来
%%原作者：Shawn Wang
%%Overleaf地址：https://www.overleaf.com/latex/templates/bei-jing-da-xue-ke-cheng-lun-wen-mo-ban/yntmqcktrzfh


\ProvidesClass{CUGReport}
\LoadClass[12pt,hyperref,a4paper,UTF8]{ctexart}
%%页边距
\RequirePackage[left=2.50cm, right=2.50cm, top=2.50cm, bottom=2.50cm]{geometry}


%%一些必要的宏包
\RequirePackage[unicode=true,colorlinks,urlcolor=blue,linkcolor=blue,bookmarksnumbered=blue]{hyperref}

\RequirePackage{latexsym,amssymb,amsmath,amsbsy,amsopn,amstext,amsthm,amsxtra,color,bm,calc,ifpdf}
\RequirePackage{siunitx} % 数学单位
\RequirePackage{graphicx}
\RequirePackage{enumerate}
\RequirePackage{fancyhdr}
\RequirePackage{listings}
\RequirePackage{multirow}
\RequirePackage{makeidx}
\RequirePackage{xcolor}
\RequirePackage{fontspec}
%\RequirePackage{subfigure}
\PassOptionsToPackage{hyphens}{url}
\RequirePackage{hyperref}
\RequirePackage{booktabs}
\RequirePackage{tcolorbox}
\RequirePackage{caption}
\RequirePackage{algorithm}    %算法宏包
\RequirePackage{algorithmic}    %算法宏包
\RequirePackage{subcaption}    %子图宏包
\RequirePackage{longtable}        %长表格宏包
\RequirePackage{natbib}            %引用相关
\RequirePackage{gbt7714}        %中文文献引用格式
\RequirePackage{hologo}            %更多Logo宏包
\RequirePackage{titletoc}            %目录宏包
%\RequirePackage{hyperref}            %设置目录与引用的超链接
%\RequirePackage[contents = ~]{background}    %水印宏包
%\RequirePackage{silence}
%\RequirePackage{everypage}
%\RequirePackage{tikz}
%\pgfdeclarelayer{background}
%\pgfsetlayers{background,main}


\RequirePackage{minted}

%增加一些包，格式化标题等
\RequirePackage{setspace,titlesec,zhnumber}
%\RequirePackage{}
%一些颜色
\definecolor{codegreen}{rgb}{0,0.6,0}
\definecolor{codegray}{rgb}{0.5,0.5,0.5}
\definecolor{codepurple}{rgb}{0.58,0,0.82}



%空格
\DeclareCaptionLabelSeparator{onespace}{~~}

% 标题深度，参考“湖南大学模板”
\setcounter{secnumdepth}{3}

% 一级标题开始时换页，参考“湖南大学模板”
\newcommand{\chapterbreak}{\clearpage}

\definecolor{myteal}{cmyk}{1,0.50,0,0} % 主题色改为地大蓝CMYK（100,50,0,0）
%% 更改摘要二字的样式
\renewcommand{\abstractname}{\textbf{\large {摘\quad 要}}}

% 图表标题文本样式 字体和字号大小
\renewcommand{\captionfont}{\songti \zihao{5}}

% 重置图片编号
\renewcommand\figurename{图}
\renewcommand{\thefigure}{\thesection-\arabic{figure}}
%修改图编号后的冒号为空格
\captionsetup[figure]{labelformat=simple, labelsep=onespace}

% 重置表格编号
\renewcommand\tablename{表}
\renewcommand{\thetable}{\thesection-\arabic{table}}

% 表格内容居中
\newcommand{\TableCenterText}[1]{
    \multicolumn{1}{c}{#1}
}

%修改表编号后的冒号为空格
\captionsetup[table]{labelformat=simple, labelsep=onespace}

% 等式编号样式
\renewcommand{\theequation}{\arabic{section}-\arabic{equation}}

%修改章节格式为中文格式，还需要继续修改，参考“湖南大学模板”
%\renewcommand{\thesection}{\chinese{section}}

%修改小节格式
%\renewcommand{thesubsection}{\chinese{subsection}}

% 代码标题
\renewcommand{\lstlistingname}{代码}

% 代码编号样式
\AtBeginDocument[B]{
    \renewcommand{\thelstlisting}{\arabic{section}-\arabic{lstlisting}}
}

% 算法编号样式
\renewcommand\thealgorithm{\arabic{section}-\arabic{algorithm}}

% 中文格式
\floatname{algorithm}{算法}  
\renewcommand{\algorithmicrequire}{\textbf{输入：}\unskip}
\renewcommand{\algorithmicensure}{\textbf{输出：}\unskip}


% 代码caption和代码块的距离 caption序号标题分隔符（空格）
\captionsetup[lstlisting]{skip=8pt, labelsep=onespace}
%20240606修改到此处，还未添加代码块格式设置代码
%代码块格式设置，缺省设置，还可以根据语言不同独立设置
\lstdefinestyle{defaultstyle}{
    backgroundcolor=\color[RGB]{254,254,254}, % 背景颜色
    commentstyle=\color{codegreen},    %注释颜色
    keywordstyle=\color{magenta},        %关键字颜色
    numberstyle=\tiny\color{codegray},    %行号格式
    stringstyle=\color{codepurple},        %字符串颜色
    basicstyle=\ttfamily \fontsize{11}{11.5}\selectfont,% 字体 字号 行距
    breaklines=true,                                %自动换行
    keepspaces=true,
    tabsize=2,
    flexiblecolumns,
    numbers=left, % 行号的位置在左边
    numberstyle=\zihao{-5}\ttfamily, % 行号字号字体
    numbersep=2em, % 设置行号的具体位置
    showspaces=false, % 是否显示空格
    showstringspaces=false,
    captionpos=b, %这段代码的名字所呈现的位置，b指的是bottom下面
    frame=ltrb,   % 边框，l代表左边单线，L代表左边双线，t指top
    % framesep=30pt, % 设置边框与代码的距离
    framexleftmargin=0pt, % 边框左扩展距离
    framexrightmargin=0pt, % 边框右扩展距离
    framextopmargin=-80pt, % 边框上扩展距离
    framexbottommargin=-40pt, % 边框下扩展距离
    framesep=1em % 设置边框与代码的距离
}

\lstset{style=defaultstyle}

% 高亮颜色定义
\RequirePackage{color}
\definecolor{lightgray}{rgb}{.9,.9,.9}
\definecolor{darkgray}{rgb}{.4,.4,.4}
\definecolor{purple}{rgb}{0.65, 0.12, 0.82}


%可固定下划线长度
\makeatletter
\newcommand\dlmu[2][4cm]{\hskip1pt\underline{\hb@xt@ #1{\hss#2\hss}}\hskip3pt}
\makeatother

% %更改主题
\hypersetup{
    colorlinks=true,
    linkcolor=black,
    filecolor=blue,
    urlcolor=magenta,
    citecolor=orange,
    linktoc=all,
}

%%对一些autoref的中文引用名作修改
\def\equationautorefname{式}
\def\footnoteautorefname{脚注}
\def\itemautorefname{项}
\def\figureautorefname{图}
\def\tableautorefname{表}
\def\partautorefname{篇}
\def\appendixautorefname{附录}
\def\chapterautorefname{章}
\def\sectionautorefname{节}
\def\subsectionautorefname{小小节}
\def\subsubsectionautorefname{subsubsection}
\def\paragraphautorefname{段落}
\def\subparagraphautorefname{子段落}
\def\FancyVerbLineautorefname{行}
\def\theoremautorefname{定理}


\newtheorem{Theorem}{定理}[section]
\newtheorem{Lemma}[Theorem]{引理}
\newtheorem{Corollary}[Theorem]{推论}
\newtheorem{Proposition}[Theorem]{命题}
\newtheorem{Definition}[Theorem]{定义}
\newtheorem{Example}[Theorem]{例}

%%文本框设置
\newcommand{\tbox}[1]{
    \begin{center}
        \begin{tcolorbox}[colback=gray!10,%gray background
                colframe=black,% black frame colour
                width=8cm,% Use 8cm total width,
                arc=1mm, auto outer arc,
                boxrule=0.5pt,
            ]
            {#1}
        \end{tcolorbox}
    \end{center}
}

\newcommand\headl[1]{\def\headl{#1}}
\newcommand\headc[1]{\def\headc{#1}}
\newcommand\headr[1]{\def\headr{#1}}
\newcommand\lessonTitle[1]{\def\lessonTitle{#1}}
\newcommand\reportTitle[1]{\def\reportTitle{#1}}
\newcommand\stuname[1]{\def\stuname{#1}}
\newcommand\stuid[1]{\def\stuid{#1}}
\newcommand\inst[1]{\def\inst{#1}}
\newcommand\major[1]{\def\major{#1}}
\renewcommand\date[1]{\def\date{#1}}

%增加一些新的命令
\renewcommand\time[1]{\def\time{#1}}        %实习时间
\newcommand\score[1]{\def\score{#1}}        %实习分数

%引用格式设置
\bibliographystyle{gbt7714-numerical}
\setlength{\bibsep}{0pt}        %参考文献内容间距
\def\bibfont{\zihao{5} \vspace{0pt}}    %参考文献标题样式

%引用样式
\setcitestyle{square}            %使用方括号
\setcitestyle{comma}            %使用逗号分隔多个引用
\setcitestyle{super}                %使用上标编号引用
%setcitestyle{numbers}            %不使用上标编号引用

\newcommand{\StartAppendix}{
    % 重置页眉
    %\newcommand{\sectionname}{附录\Alph{section}}

    % 重置一级标题记数
    \setcounter{section}{0}
    
    % 图目录图号样式
%    \titlecontents{figure}
%        [0.5cm] % 左间距
%        {\songti \zihao{-4}} % 标题样式
%        {\figurename~\thecontentslabel\quad} % 标题格式
 %       {}
%        {\titlerule*[0.5em]{·}\contentspage} % 指引线样式
%        [\addvspace{0pt}] % 每条间距

    % 表目录表号样式
%    \titlecontents{table}
 %       [0.5cm] % 左间距
%        {\songti \zihao{-4}} % 标题样式
%        {\tablename~\thecontentslabel\quad} % 标题格式
%        {}
%        {\titlerule*[0.5em]{·}\contentspage} % 指引线样式
%        [\addvspace{0pt}] % 每条间距

    % 重置一级标题目录样式
    \titlecontents{section}[0pt]{\bfseries \songti \zihao{-4}} % 缩进、字体字号
    {附录 \thecontentslabel~~}{}
    {\titlerule*[0.5em]{·}\contentspage} % 连接符样式、密度
    
    % 重置一级标题样式
    \titleformat{\section}[block]
    {\vspace*{-20pt} \centering \zihao{-2} \songti \bfseries} % 标题和上文距离 字号 字体
    {附录 \thesection~~} % 标题序号样式
    {5pt} % 标题序号和标题文本距离
    {\vspace{10pt}} % 标题和下文距离

    % 重置二级标题样式
    \titleformat{\subsection}[block]
    {\vspace{0pt} \zihao{4} \heiti} % 标题和上文的距离 字号 字体
    {\bfseries \thesubsection~~} % 标题序号样式
    {5pt} % 标题序号和标题文本距离
    {\vspace{0pt}} % 标题和下文距离
    
    % 重置三级标题样式
    \titleformat{\subsubsection}
    {\vspace{0pt} \zihao{4} \heiti} % 标题和上文的距离 字号 字体
    {\bfseries \thesubsubsection~~} % 标题序号样式
    {5pt} % 标题序号和标题文本距离
    {\vspace{0pt}} % 标题和下文距离
    

    % 重置一级标题编号
    \renewcommand{\thesection}{\texorpdfstring{}{附录 }\Alph{section}\texorpdfstring{}{  }}

    % 重置二级标题编号
    \renewcommand{\thesubsection}{\thesection.\arabic{subsection}}
    
    % 重置三级标题编号
    \renewcommand{\thesubsubsection}{\thesection.\arabic{subsection}.\arabic{subsubsection}}
    

    % 重置公式编号
    \renewcommand{\theequation}{\thesection-\arabic{equation}}

    % 代码编号样式
    \renewcommand{\thelstlisting}{\thesection-\arabic{lstlisting}}

}


% 致谢
\newcommand{\StartAcknowledgements}{

    \def\currentSectionInfo{致谢} % 自定义当前章节名

    %\NoBgThispage % 取消水印
    
    % 重置图片编号
    \renewcommand\figurename{图}
    \renewcommand{\thefigure}{\thesection-\arabic{figure}}
    
    % 重置表格编号
    \renewcommand\tablename{表}
    \renewcommand{\thetable}{\thesection-\arabic{table}}

    % 换页
    \clearpage

    \phantomsection

    % 添加致谢到目录
    \addcontentsline{toc}{section}{致~~~~谢}

    % 致谢标题
    \vspace*{-20pt}
    \begin{center}
        \zihao{-2} \songti \bfseries 致~~~~谢
    \end{center}
    \vspace*{10pt}

}



    

%%页眉设置
\pagestyle{fancy}
\fancyhead[L]{\fangsong {\headl}}
\fancyhead[C]{\fangsong {\headc}}
\fancyhead[R]{\fangsong {\headr}}
%% 去掉页眉横线
% \renewcommand{\headrulewidth}{0pt}

%% 常用图片文件夹路径
\graphicspath{{figures/}{figure/}{pictures/}{picture/}{pic/}{pics/}{image/}{images/}}

%%在这里可更改封面logo
\newcommand{\cover}{
    %%封面校名logo，图片都在figures文件夹里
     \begin{center}
        \begin{figure}
            \centering
            \vspace{1cm}
            \includegraphics[width=0.6\textwidth]{figures/xinxiaoming-hei.png}
        \end{figure}
    %校徽    
    %\begin{center}
        \begin{figure}
            \centering
            \vspace{1cm}
            \includegraphics[width=0.2\textwidth]{figures/xinxiaohui-1952(2).png}
        \end{figure}

        \vspace*{\fill}

        \heiti \Huge \textbf{{\lessonTitle}} \par
        \vspace{1cm}
        \heiti \Large {\underline{\reportTitle}} \par
        \vspace{2cm}

        \vspace{0.5cm}
        \kaishu\Large 姓\ 名\ \dlmu[9cm]{\stuname} \qquad \\ %姓名 
        \vspace{0.5cm}
        \kaishu\Large 学\ 号\ \dlmu[9cm]{\stuid} \qquad  \\ %学号
        \vspace{0.5cm}
        \kaishu\Large 学\ 院\ \dlmu[9cm]{\inst} \qquad \\ %院所
        \vspace{0.5cm}
        \kaishu\Large 专\ 业\ \dlmu[9cm]{\major} \\ %专业
        \vspace{0.5cm}
        \kaishu\Large 时\ 间\ \dlmu[9cm]{\time} \\   %时间
        \vspace{0.5cm}
        \kaishu\Large 成\ 绩\ \dlmu[9cm]{\score}     %成绩


        \vspace*{\fill}

        \large \date
    \end{center}
}

