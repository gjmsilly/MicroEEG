/ *   U S E R   C O D E   B E G I N   H e a d e r   * /  
 / * *  
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
     *   @ f i l e                       :   m a i n . h  
     *   @ b r i e f                     :   H e a d e r   f o r   m a i n . c   f i l e .  
     *                                       T h i s   f i l e   c o n t a i n s   t h e   c o m m o n   d e f i n e s   o f   t h e   a p p l i c a t i o n .  
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
     *   @ a t t e n t i o n  
     *  
     *   < h 2 > < c e n t e r > & c o p y ;   C o p y r i g h t   ( c )   2 0 2 0   S T M i c r o e l e c t r o n i c s .  
     *   A l l   r i g h t s   r e s e r v e d . < / c e n t e r > < / h 2 >  
     *  
     *   T h i s   s o f t w a r e   c o m p o n e n t   i s   l i c e n s e d   b y   S T   u n d e r   B S D   3 - C l a u s e   l i c e n s e ,  
     *   t h e   " L i c e n s e " ;   Y o u   m a y   n o t   u s e   t h i s   f i l e   e x c e p t   i n   c o m p l i a n c e   w i t h   t h e  
     *   L i c e n s e .   Y o u   m a y   o b t a i n   a   c o p y   o f   t h e   L i c e n s e   a t :  
     *                                                 o p e n s o u r c e . o r g / l i c e n s e s / B S D - 3 - C l a u s e  
     *  
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *  
     * /  
 / *   U S E R   C O D E   E N D   H e a d e r   * /  
  
 / *   D e f i n e   t o   p r e v e n t   r e c u r s i v e   i n c l u s i o n   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - * /  
 # i f n d e f   _ _ M A I N _ H  
 # d e f i n e   _ _ M A I N _ H  
  
 # i f d e f   _ _ c p l u s p l u s  
 e x t e r n   " C "   {  
 # e n d i f  
  
 / *   I n c l u d e s   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - * /  
 # i n c l u d e   " s t m 3 2 f 4 x x _ h a l . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ a d c . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ d m a . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ i 2 c . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ r c c . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ b u s . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ s y s t e m . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ e x t i . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ c o r t e x . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ u t i l s . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ p w r . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ r t c . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ s p i . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ t i m . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ u s a r t . h "  
 # i n c l u d e   " s t m 3 2 f 4 x x _ l l _ g p i o . h "  
  
 / *   P r i v a t e   i n c l u d e s   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - * /  
 / *   U S E R   C O D E   B E G I N   I n c l u d e s   * /  
  
 / *   U S E R   C O D E   E N D   I n c l u d e s   * /  
  
 / *   E x p o r t e d   t y p e s   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - * /  
 / *   U S E R   C O D E   B E G I N   E T   * /  
  
 / *   U S E R   C O D E   E N D   E T   * /  
  
 / *   E x p o r t e d   c o n s t a n t s   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - * /  
 / *   U S E R   C O D E   B E G I N   E C   * /  
 e x t e r n   u i n t 1 6 _ t   S Y S _ E v e n t ; 	 	 	 	 	 	 	 	 	 / / ! <   �|�~�r`�N�N  -   @ r e f   S y s t e m   e v e n t s  
  
 / *   U S E R   C O D E   E N D   E C   * /  
  
 / *   E x p o r t e d   m a c r o   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - * /  
 / *   U S E R   C O D E   B E G I N   E M   * /  
  
 / /   S y s t e m   e v e n t s  
 # d e f i n e   A T T R _ C H A N G E _ E V T 	 	 	 	 	 (   1   < <   0   ) 	 / / ! <   ^\'`<P�SS 
 # d e f i n e   T C P _ R E C V _ E V T 	 	 	 	 	 	 (   1   < <   1   )   	 / / ! <   T C P �z�S�c6e0R N'^ 
 # d e f i n e   T C P _ P R O C E S S C L P _ E V T 	 	 	 (   1   < <   2   )   	 / / ! <   T C P '^OS��Yt�[�k 
 # d e f i n e   T C P _ S E N D _ E V T                         (   1   < <   3   ) 	 / / ! <   T C P �z�S�VY�[b 
 # d e f i n e   U D P _ R E C V _ E V T 	 	 	 	 	 	 (   1   < <   4   ) 	 / / ! <   U D P �z�S�c6e0R N'^ 
 # d e f i n e   U D P _ D T P R O C E S S C L P _ E V T 	 	 (   1   < <   5   ) 	 / / ! <   U D P penc'^OS��Yt�[�k 
 # d e f i n e   U D P _ T R G P R O C E S S C L P _ E V T 	 	 (   1   < <   6   ) 	 / / ! <   U D P �N�N'^OS��Yt�[�k 
 # d e f i n e   E E G _ I M P _ S T A R T 	 	 	 	 	 	 (   1   < <   7   ) 	 / / ! <   ;��b�hKm _�Y 
 # d e f i n e   E E G _ D A T A _ S T A R T _ E V T 	 	 	 (   1   < <   8   ) 	 / / ! <    NSA D penc _�YǑƖ 
 # d e f i n e   E E G _ D A T A _ A C Q _ E V T 	 	 	 	 (   1   < <   9   ) 	 / / ! <    NSA D pencǑƖ-N 
 # d e f i n e   E E G _ D A T A _ C P L _ E V T 	 	 	 	 (   1   < <   1 0   ) 	 / / ! <    NSA D pencǑƖ�[b 
 # d e f i n e   E E G _ S T O P _ E V T 	 	 	 	 	 	 (   1   < <   1 1   ) 	 / / ! <   A D penc�f\PǑƖ 
 # d e f i n e   T R I G G E R _ E V T 	 	 	 	 	 	 	 ( 	 1 	 < <   1 2   ) 	 / / ! <   h~{�N�N 
 # d e f i n e   P O W E R D O W N _ E V T 	 	 	 	 	 	 ( 	 1 	 < <   1 3   ) 	 / / ! <   _8^�e5u�N�N 
 # d e f i n e   S O C K E T D O W N _ E V T 	 	 	 	 	 ( 	 1 	 < <   1 4   ) 	 / / ! <   w 5 5 0 0 ޏ�c�e _�N�N 
  
 / *   U S E R   C O D E   E N D   E M   * /  
  
 / *   E x p o r t e d   f u n c t i o n s   p r o t o t y p e s   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - * /  
 v o i d   E r r o r _ H a n d l e r ( v o i d ) ;  
  
 / *   U S E R   C O D E   B E G I N   E F P   * /  
  
 / *   U S E R   C O D E   E N D   E F P   * /  
  
 / *   P r i v a t e   d e f i n e s   - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - * /  
 # d e f i n e   E R R _ L E D 2 _ P i n   L L _ G P I O _ P I N _ 2  
 # d e f i n e   E R R _ L E D 2 _ G P I O _ P o r t   G P I O E  
 # d e f i n e   E R R _ L E D 1 _ P i n   L L _ G P I O _ P I N _ 3  
 # d e f i n e   E R R _ L E D 1 _ G P I O _ P o r t   G P I O E  
 # d e f i n e   A C Q _ L E D 2 _ P i n   L L _ G P I O _ P I N _ 4  
 # d e f i n e   A C Q _ L E D 2 _ G P I O _ P o r t   G P I O E  
 # d e f i n e   A C Q _ L E D 1 _ P i n   L L _ G P I O _ P I N _ 5  
 # d e f i n e   A C Q _ L E D 1 _ G P I O _ P o r t   G P I O E  
 # d e f i n e   P W R _ L E D 1 _ P i n   L L _ G P I O _ P I N _ 6  
 # d e f i n e   P W R _ L E D 1 _ G P I O _ P o r t   G P I O E  
 # d e f i n e   P W R _ L E D 2 _ P i n   L L _ G P I O _ P I N _ 1 3  
 # d e f i n e   P W R _ L E D 2 _ G P I O _ P o r t   G P I O C  
 # d e f i n e   L a b e l _ T G R _ I n 1 _ P i n   L L _ G P I O _ P I N _ 3  
 # d e f i n e   L a b e l _ T G R _ I n 1 _ G P I O _ P o r t   G P I O A  
 # d e f i n e   L a b e l _ T G R _ O u t _ P i n   L L _ G P I O _ P I N _ 5  
 # d e f i n e   L a b e l _ T G R _ O u t _ G P I O _ P o r t   G P I O A  
 # d e f i n e   L a b e l _ T G R _ I n 2 _ P i n   L L _ G P I O _ P I N _ 6  
 # d e f i n e   L a b e l _ T G R _ I n 2 _ G P I O _ P o r t   G P I O A  
 # d e f i n e   n C h a r g e E N _ P i n   L L _ G P I O _ P I N _ 4  
 # d e f i n e   n C h a r g e E N _ G P I O _ P o r t   G P I O C  
 # d e f i n e   P G a u _ P i n   L L _ G P I O _ P I N _ 5  
 # d e f i n e   P G a u _ G P I O _ P o r t   G P I O C  
 # d e f i n e   K e y P a d _ I O 1 _ P i n   L L _ G P I O _ P I N _ 7  
 # d e f i n e   K e y P a d _ I O 1 _ G P I O _ P o r t   G P I O E  
 # d e f i n e   K e y P a d _ I O 0 _ P i n   L L _ G P I O _ P I N _ 8  
 # d e f i n e   K e y P a d _ I O 0 _ G P I O _ P o r t   G P I O E  
 # d e f i n e   E X T M _ I O 3 _ P i n   L L _ G P I O _ P I N _ 1 2  
 # d e f i n e   E X T M _ I O 3 _ G P I O _ P o r t   G P I O E  
 # d e f i n e   E X T M _ I O 2 _ P i n   L L _ G P I O _ P I N _ 1 3  
 # d e f i n e   E X T M _ I O 2 _ G P I O _ P o r t   G P I O E  
 # d e f i n e   E X T M _ I O 1 _ P i n   L L _ G P I O _ P I N _ 1 4  
 # d e f i n e   E X T M _ I O 1 _ G P I O _ P o r t   G P I O E  
 # d e f i n e   E X T M _ I O 0 _ P i n   L L _ G P I O _ P I N _ 1 5  
 # d e f i n e   E X T M _ I O 0 _ G P I O _ P o r t   G P I O E  
 # d e f i n e   M o d 5 _ S C K _ P i n   L L _ G P I O _ P I N _ 1 3  
 # d e f i n e   M o d 5 _ S C K _ G P I O _ P o r t   G P I O B  
 # d e f i n e   M o d 5 _ M I S O _ P i n   L L _ G P I O _ P I N _ 1 4  
 # d e f i n e   M o d 5 _ M I S O _ G P I O _ P o r t   G P I O B  
 # d e f i n e   M o d 5 _ M O S I _ P i n   L L _ G P I O _ P I N _ 1 5  
 # d e f i n e   M o d 5 _ M O S I _ G P I O _ P o r t   G P I O B  
 # d e f i n e   M o d 5 _ n C S _ P i n   L L _ G P I O _ P I N _ 8  
 # d e f i n e   M o d 5 _ n C S _ G P I O _ P o r t   G P I O D  
 # d e f i n e   M o d 5 _ S T A R T _ P i n   L L _ G P I O _ P I N _ 9  
 # d e f i n e   M o d 5 _ S T A R T _ G P I O _ P o r t   G P I O D  
 # d e f i n e   M o d 5 _ n R E S E T _ P i n   L L _ G P I O _ P I N _ 1 0  
 # d e f i n e   M o d 5 _ n R E S E T _ G P I O _ P o r t   G P I O D  
 # d e f i n e   M o d 5 _ n P W D N _ P i n   L L _ G P I O _ P I N _ 1 1  
 # d e f i n e   M o d 5 _ n P W D N _ G P I O _ P o r t   G P I O D  
 # d e f i n e   T R G P W R _ E N _ P i n   L L _ G P I O _ P I N _ 7  
 # d e f i n e   T R G P W R _ E N _ G P I O _ P o r t   G P I O C  
 # d e f i n e   C A N _ S _ P i n   L L _ G P I O _ P I N _ 8  
 # d e f i n e   C A N _ S _ G P I O _ P o r t   G P I O C  
 # d e f i n e   W 5 5 0 0 _ M O S I _ P i n   L L _ G P I O _ P I N _ 9  
 # d e f i n e   W 5 5 0 0 _ M O S I _ G P I O _ P o r t   G P I O C  
 # d e f i n e   C A N _ D _ P i n   L L _ G P I O _ P I N _ 8  
 # d e f i n e   C A N _ D _ G P I O _ P o r t   G P I O A  
 # d e f i n e   W 5 5 0 0 _ n R S T _ P i n   L L _ G P I O _ P I N _ 1 5  
 # d e f i n e   W 5 5 0 0 _ n R S T _ G P I O _ P o r t   G P I O A  
 # d e f i n e   W 5 5 0 0 _ M I S O _ P i n   L L _ G P I O _ P I N _ 1 0  
 # d e f i n e   W 5 5 0 0 _ M I S O _ G P I O _ P o r t   G P I O C  
 # d e f i n e   W 5 5 0 0 _ n I N T _ P i n   L L _ G P I O _ P I N _ 0  
 # d e f i n e   W 5 5 0 0 _ n I N T _ G P I O _ P o r t   G P I O D  
 # d e f i n e   W 5 5 0 0 _ n I N T _ E X T I _ I R Q n   E X T I 0 _ I R Q n  
 # d e f i n e   M o d 1 _ n C S _ P i n   L L _ G P I O _ P I N _ 1  
 # d e f i n e   M o d 1 _ n C S _ G P I O _ P o r t   G P I O D  
 # d e f i n e   M o d 2 _ n C S _ P i n   L L _ G P I O _ P I N _ 2  
 # d e f i n e   M o d 2 _ n C S _ G P I O _ P o r t   G P I O D  
 # d e f i n e   W 5 5 0 0 _ S C L K _ P i n   L L _ G P I O _ P I N _ 3  
 # d e f i n e   W 5 5 0 0 _ S C L K _ G P I O _ P o r t   G P I O D  
 # d e f i n e   M o d c _ n D R D Y _ P i n   L L _ G P I O _ P I N _ 4  
 # d e f i n e   M o d c _ n D R D Y _ G P I O _ P o r t   G P I O D  
 # d e f i n e   M o d 3 _ n C S _ P i n   L L _ G P I O _ P I N _ 5  
 # d e f i n e   M o d 3 _ n C S _ G P I O _ P o r t   G P I O D  
 # d e f i n e   M o d 4 _ n C S _ P i n   L L _ G P I O _ P I N _ 6  
 # d e f i n e   M o d 4 _ n C S _ G P I O _ P o r t   G P I O D  
 # d e f i n e   M o d 1 _ n D R D Y _ P i n   L L _ G P I O _ P I N _ 7  
 # d e f i n e   M o d 1 _ n D R D Y _ G P I O _ P o r t   G P I O D  
 # d e f i n e   W 5 5 0 0 _ n C S _ P i n   L L _ G P I O _ P I N _ 6  
 # d e f i n e   W 5 5 0 0 _ n C S _ G P I O _ P o r t   G P I O B  
 # d e f i n e   M o d _ S T A R T _ P i n   L L _ G P I O _ P I N _ 8  
 # d e f i n e   M o d _ S T A R T _ G P I O _ P o r t   G P I O B  
 # d e f i n e   M o d _ n R E S E T _ P i n   L L _ G P I O _ P I N _ 9  
 # d e f i n e   M o d _ n R E S E T _ G P I O _ P o r t   G P I O B  
 # d e f i n e   M o d _ n P W D N _ P i n   L L _ G P I O _ P I N _ 0  
 # d e f i n e   M o d _ n P W D N _ G P I O _ P o r t   G P I O E  
 # d e f i n e   M o d s _ n D R D Y _ P i n   L L _ G P I O _ P I N _ 1  
 # d e f i n e   M o d s _ n D R D Y _ G P I O _ P o r t   G P I O E  
 # d e f i n e   M o d s _ n D R D Y _ E X T I _ I R Q n   E X T I 1 _ I R Q n  
 / *   U S E R   C O D E   B E G I N   P r i v a t e   d e f i n e s   * /  
  
 # d e f i n e   E R R _ L E D 1 _ O F F     	 L L _ G P I O _ S e t O u t p u t P i n     ( G P I O E ,   E R R _ L E D 1 _ P i n ) ;  
 # d e f i n e   E R R _ L E D 1 _ O N       	 L L _ G P I O _ R e s e t O u t p u t P i n ( G P I O E ,   E R R _ L E D 1 _ P i n ) ;  
 # d e f i n e   E R R _ L E D 2 _ O F F 	 	 L L _ G P I O _ S e t O u t p u t P i n     ( G P I O E ,   E R R _ L E D 2 _ P i n ) ;  
 # d e f i n e   E R R _ L E D 2 _ O N       	 L L _ G P I O _ R e s e t O u t p u t P i n ( G P I O E ,   E R R _ L E D 2 _ P i n ) ;  
 # d e f i n e   A C Q _ L E D 1 _ O F F     	 L L _ G P I O _ S e t O u t p u t P i n     ( G P I O E ,   A C Q _ L E D 1 _ P i n ) ;  
 # d e f i n e   A C Q _ L E D 1 _ O N       	 L L _ G P I O _ R e s e t O u t p u t P i n ( G P I O E ,   A C Q _ L E D 1 _ P i n ) ;  
 # d e f i n e   A C Q _ L E D 2 _ O F F     	 L L _ G P I O _ S e t O u t p u t P i n     ( G P I O E ,   A C Q _ L E D 2 _ P i n ) ;  
 # d e f i n e   A C Q _ L E D 2 _ O N       	 L L _ G P I O _ R e s e t O u t p u t P i n ( G P I O E ,   A C Q _ L E D 2 _ P i n ) ;  
 # d e f i n e   A C Q _ L E D 1 _ T O G G L E 	 L L _ G P I O _ T o g g l e P i n ( G P I O E ,   A C Q _ L E D 1 _ P i n ) ;  
 # d e f i n e   P W R _ L E D 1 _ O F F     	 L L _ G P I O _ S e t O u t p u t P i n     ( G P I O E ,   P W R _ L E D 1 _ P i n ) ;  
 # d e f i n e   P W R _ L E D 1 _ O N       	 L L _ G P I O _ R e s e t O u t p u t P i n ( G P I O E ,   P W R _ L E D 1 _ P i n ) ;  
 # d e f i n e   P W R _ L E D 2 _ O F F     	 L L _ G P I O _ S e t O u t p u t P i n     ( G P I O C ,   P W R _ L E D 2 _ P i n ) ;  
 # d e f i n e   P W R _ L E D 2 _ O N       	 L L _ G P I O _ R e s e t O u t p u t P i n ( G P I O C ,   P W R _ L E D 2 _ P i n ) ;  
  
 / *   U S E R   C O D E   E N D   P r i v a t e   d e f i n e s   * /  
  
 # i f d e f   _ _ c p l u s p l u s  
 }  
 # e n d i f  
  
 # e n d i f   / *   _ _ M A I N _ H   * /  
  
 / * * * * * * * * * * * * * * * * * * * * * * * *   ( C )   C O P Y R I G H T   S T M i c r o e l e c t r o n i c s   * * * * * E N D   O F   F I L E * * * * /  
 