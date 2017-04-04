m4_dnl m4 -> markdown -> html website macros
m4_dnl
m4_dnl Also see makefile, makefile.project, and makefile.dir
m4_dnl
m4_dnl Please put site-specific macros in site-macros.m4
m4_dnl
m4_dnl Any uncommented text in this file, including whitespace and newlines,
m4_dnl will be passed through to generated files.  This caues a problem with
m4_dnl generated markdown files which want to have % Page Title as the very
m4_dnl first line.  Thus, make sure this file only contains m4 macros that 
m4_dnl expand to nothing, that every newline is prefixed with m4_dnl, and 
m4_dnl that there is no extra whitespace.
m4_dnl
m4_changecom(`<!--',`-->')m4_dnl
m4_define(`SITE_NAME', `TBD')m4_dnl please redefine in site-macros.m4
m4_define(`SITE_ACRONYM', `TBD')m4_dnl please redefine in site-macros.m4
m4_define(`SITE_HOME', `TBD')m4_dnl please redefine in site-macros.m4
m4_define(`SITE_SIDEBAR',`')m4_dnl please redefine in site-macros.m4
m4_include(WWW_INC/`site-macros.m4')m4_dnl
m4_define(`PAGE_TITLE', `% m4_ifelse(`$1',`',`SITE_NAME'm4_ifelse(SITE_ACRONYM,`',`',` (SITE_ACRONYM)'),`m4_ifelse(SITE_ACRONYM,`',`',`SITE_ACRONYM: ')$1')')m4_dnl
m4_define(`RFIG',
`<table style="float: right" border="0"><tr><td>
<a href="$1">
<img src="$1" border="0"/>
</a></td></tr><tr><td align="center" style="font-size: 70%">$2</td></tr></table>')m4_dnl
m4_define(`RFIGW',
`<table style="float: right" border="0"><tr><td>
<a href="$1">
<img src="$1" width="$3" border="0"/>
</a></td></tr><tr><td align="center" style="font-size: 70%">$2</td></tr></table>')m4_dnl
m4_define(`STRIKE', `<span style="text-decoration: line-through">$1</span>')m4_dnl
m4_define(`COLOR', `<span style="color: $1">$2</span>')m4_dnl
m4_define(`SMALL', `<span style="font-size: 70%">$1</span>')m4_dnl
m4_define(`LARGE', `<span style="font-size: 130%">$1</span>')m4_dnl
m4_define(`CODE', `<code>$1</code>')m4_dnl
m4_define(`DISCLAIMER', `THIS INFORMATION AND/OR SOFTWARE IS PROVIDED BY THE AUTHOR "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS INFORMATION AND/OR SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.')m4_dnl
m4_dnl THUMB(basename, ext, thumbsize, title, extra)
m4_define(`THUMB',`<a href="$1.$2"><img src="$1-$3.$2" title="$4" $5/></a>')m4_dnl
m4_define(`IMG_SM', <table border="0"><tr><td align="center"><table><tr><td align="right"><font size="0"><a href="$1-med.$2">medium</a>|<a href="$1.$2">high</a> resolution</font></td></tr><tr><td align="center"><a href="$1-med.$2"><img src="$1-small.$2"/></a></td></tr></table></td></tr><tr><td align="center">$3</td></tr></table>)m4_dnl
m4_define(`IMG_S', <table border="0"><tr><td align="center"><table><tr><td align="right"><font size="0"><a href="$1.$2">high</a> resolution</font></td></tr><tr><td align="center"><a href="$1.$2"><img src="$1-small.$2"/></a></td></tr></table></td></tr><tr><td align="center">$3</td></tr></table>)m4_dnl
m4_define(`PDF_THUMB', <table border="0"><tr><td align="center"><a href="$1.pdf"><img src="$1.png"/></a></td></tr><tr><td align="center">$2</td></tr></table>)m4_dnl
m4_define(`GNU_GPL_L',`<a href="http://www.gnu.org">GNU</a> <a href="http://www.gnu.org/copyleft/gpl.html">GPL</a>')m4_dnl
m4_define(`M4_L',`<a href="http://www.gnu.org/software/m4/">M4</a>')m4_dnl
m4_define(`MARKDOWN_L',`<a href="http://johnmacfarlane.net/pandoc/">markdown</a>')m4_dnl
m4_define(`BR_CLEAR_ALL',`<br clear="all"/>')m4_dnl
m4_define(`HR_DOTTED',`<hr style="width: 100%; clear: both; border-top: 1px dotted black; border-right: none; border-bottom: none; border-left: none;"/>')m4_dnl
m4_define(`APOS',`$1&#39;$2')m4_dnl
m4_dnl JS_PLAYER(basename, width, height, extra-style, dir, options)
m4_define(`JS_PLAYER',<video id="$5-$1" class="video-js vjs-default-skin" controls preload="auto" width="$2" height="$3" poster="$5/$1/$1.png" data-setup='{$6}'> <source src="$5/$1/$1.mp4" type="video/mp4" /> <source src="$5/$1/$1.webm" type="video/webm" /> <source src="$5/$1/$1.ogv" type="video/ogg" /> </video>)m4_dnl
m4_dnl DEMO(title, basename, width, height, caption, extra-style, dir, options)
m4_define(`DEMO',`<table style="margin: 20px; $6"><tr>m4_ifelse(`$1',`',`',`<td align="left"><big>$1</big></td>')<td align="right" style="font-size:85%"><a href="$7/$2/$2.ogv">ogv</a>|<a href="$7/$2/$2.mp4">mp4</a>|<a href="$7/$2/$2.webm">webm</a></td></tr><tr><td colspan="2" m4_ifelse(`$1',`',`',`style="border-top: 2px solid #9999cc"')>JS_PLAYER($2, $3, $4, `', $7, $8)</td></tr><tr><td colspan="2" width="$3" valign="top"><p align="justify" style="text-indent: 0em">$5</p></td></tr></table>')m4_dnl
m4_dnl<a name="$7-$2"/a>m4_dnl
m4_define(`SEPARATOR', `<h$2 class="m4_ifelse(`$4',`',`separator',`$4')"><a name="m4_ifelse(`$3',`',`$1',`$3')"/a>$1</h$2>')m4_dnl
m4_define(`CITE', `[<a href="#$1">$1</a>]')m4_dnl
m4_dnl REF(id, authors, name, where, when)
m4_define(`REF', `<tr><td valign="top"><a name="$1">$1</a></td><td>$2.  <i>$3</i>. $4, $5.</td></tr>')m4_dnl
m4_define(`CLINK', `<a href="$1#$2">$3</a>')m4_dnl
