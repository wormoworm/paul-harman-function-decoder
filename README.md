# DCC function decoder by Paul Harman (modified)

## What is it?
The repository hosts a modified version of Paul Harman's excellent DCC function decoder software for the PIC 12F629 (and similar) microcontrollers, and its compiled hex counterpart.

## Why is it?
The most recent version of the decoder software I could find (v2.19) contains a bug whereby commands sent to DCC long addresses that have bit 11 set (technically bit 3 of CV17) are ignored. Whilst checking on the [DIY Decoder forum](https://diydecoder.proboards.com/), I found that other users had encountered this issue back in 2014. There is a good discussion in [this thread](https://diydecoder.proboards.com/thread/70/long-address-bug) that details the issue, together with a suggestion for a fix. This repository contains the result of applying that fix to the publically-available v2.19 software. I have chosen to host this on GitHub so that others can benenfit from this fix also.

## What is the change?
The change (originally suggested by user _Trond_ in the thread), is to remove lines 1170 and 1171 from the program. This causes the decoder to not ignore addresses with bit 3 in CV17 set. I believe it might be contrary to the NMRA DCC spec to skip this check, but I personally don't see the harm in doing this. I'd rather have a decoder that can be programmed with any long address than one that cannot!

## Notes
+ This repo contains the assembler and hex files for both the original v2.19 code, and the modified v2.19 code with the fix.
+ For more information about this excellent project, please visit the [project homepage](https://dccdiy.org.uk/function.html) or the [forum](https://diydecoder.proboards.com).